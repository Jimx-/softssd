#include "xparameters.h"
#include <stdio.h>
#include <stdlib.h>
#include "xil_printf.h"
#include "xil_mmu.h"
#include "sleep.h"
#include "xil_cache.h"
#include <errno.h>

#include <ecc_config.h>
#include <flash_config.h>
#include <ecc.h>
#include <proto.h>
#include <ringq.h>
#include <utils.h>

#include "bch_engine.h"

#define PS_DDR_LOW_BASE_ADDR 0x1100000UL
#define PS_DDR_LOW_LIMIT     0x80000000UL /* 2G */

#define ATCM_BASE_ADDR XPAR_PSU_R5_1_ATCM_S_AXI_BASEADDR
#define ATCM_LIMIT     (XPAR_PSU_R5_1_ATCM_S_AXI_HIGHADDR + 1)
#define BTCM_BASE_ADDR XPAR_PSU_R5_1_BTCM_S_AXI_BASEADDR
#define BTCM_LIMIT     (XPAR_PSU_R5_1_BTCM_S_AXI_HIGHADDR + 1)

static struct ringq ecc_ringq;
static int send_ipi;

static struct bch_engine bch_engine;

static void handle_ecc_task(struct ecc_task* task);

static int dequeue_requests(void)
{
    uint32_t elem;
    int found = FALSE;
    struct ecc_task* task = NULL;

    ringq_read_avail_tail(&ecc_ringq);

    while (ringq_get_avail(&ecc_ringq, &elem)) {
        found = TRUE;
        task = (struct ecc_task*)(uintptr_t)elem;
        Xil_DCacheInvalidateRange((UINTPTR)task, sizeof(*task));
        Xil_DCacheInvalidateRange((UINTPTR)task->data, task->length);

        /* For correction, we also need to invalidate the code buffer. */
        if (task->type == ECT_CORRECT)
            Xil_DCacheInvalidateRange((UINTPTR)task->code, task->code_length);

        handle_ecc_task(task);
    }

    return found;
}

static void enqueue_response(struct ecc_task* task)
{
    Xil_AssertVoid(task->completed);

    Xil_DCacheFlushRange((UINTPTR)task, sizeof(*task));

    if (task->type == ECT_CORRECT)
        Xil_DCacheFlushRange((UINTPTR)task->data, task->length);

    if (task->type == ECT_CALC)
        Xil_DCacheFlushRange((UINTPTR)task->code, task->code_length);

    ringq_add_used(&ecc_ringq, (uint32_t)(uintptr_t)task);
    ringq_write_used_tail(&ecc_ringq);
    send_ipi = TRUE;
}

static void notify_task_complete(struct ecc_task* task, int status)
{
    task->status = status;
    task->completed = TRUE;
    enqueue_response(task);
}

static int handle_calc_task(const u8* data, size_t data_length, u8* code,
                            uint32_t* code_length, size_t offset)
{
    unsigned int i;
    int start = offset / BCH_BLOCK_SIZE;
    int nblocks = ((data_length + BCH_BLOCK_SIZE - 1) / BCH_BLOCK_SIZE);
    size_t code_size = (start + nblocks) * BCH_CODE_SIZE;

    if (*code_length < code_size) {
        return -ENOMEM;
    }

    for (i = start; i < start + nblocks; i++) {
        bch_engine_calculate(&bch_engine, data + i * BCH_BLOCK_SIZE,
                             code + i * BCH_CODE_SIZE);
    }

    *code_length = code_size;
    return 0;
}

static int handle_correct_task(u8* data, size_t data_length, const u8* code,
                               uint32_t* code_length, uint64_t err_bitmap)
{
    unsigned int i;
    int nblocks = ((data_length + BCH_BLOCK_SIZE - 1) / BCH_BLOCK_SIZE);
    size_t code_size = nblocks * BCH_CODE_SIZE;
    unsigned char* calc_ecc =
        (unsigned char*)BTCM_BASE_ADDR; /* Use BTCM to hold calculated ECC. */
    int r;

    if (unlikely(!err_bitmap)) return 0;

    if (*code_length < code_size) {
        return -ENOMEM;
    }

    for (i = 0; i < nblocks; i++) {
        if (err_bitmap & (1ULL << i)) {
            bch_engine_calculate(&bch_engine, data + i * BCH_BLOCK_SIZE,
                                 calc_ecc + i * BCH_CODE_SIZE);
        }
    }

    for (i = 0; i < nblocks; i++) {
        if (err_bitmap & (1ULL << i)) {
            r = bch_engine_correct(&bch_engine, data + i * BCH_BLOCK_SIZE,
                                   code + i * BCH_CODE_SIZE,
                                   calc_ecc + i * BCH_CODE_SIZE);
            if (r < 0) break;
        }
    }

    *code_length = code_size;

    return r;
}

static void handle_ecc_task(struct ecc_task* task)
{
    int r;
    int status;

    switch (task->type) {
    case ECT_CALC:
        r = handle_calc_task((u8*)(uintptr_t)task->data, task->length,
                             (u8*)(uintptr_t)task->code, &task->code_length,
                             task->offset);
        break;
    case ECT_CORRECT:
        r = handle_correct_task((u8*)(uintptr_t)task->data, task->length,
                                (u8*)(uintptr_t)task->code, &task->code_length,
                                task->err_bitmap);
        break;
    }

    switch (r) {
    case 0:
        status = ETS_OK;
        break;
    case -ENOMEM:
        status = ETS_NOSPC;
        break;
    case -EBADMSG:
        status = ETS_DEC_ERROR;
        break;
    default:
        if (r > 0) status = ETS_OK;
        break;
    }

    notify_task_complete(task, status);
}

int main(void)
{
    void* ecc_ringq_buf;
    int r;

    perfcounter_init(TRUE, FALSE);

    ipi_setup();

    ecc_ringq_buf = (void*)(uintptr_t)PS_DDR_LOW_BASE_ADDR;
    Xil_SetTlbAttributes((UINTPTR)ecc_ringq_buf,
                         NORM_SHARED_NCACHE | PRIV_RW_USER_NA);

    ringq_init(&ecc_ringq, ecc_ringq_buf, 1 << 20);

    r = bch_engine_init(&bch_engine, BCH_BLOCK_SIZE, BCH_CODE_SIZE);
    if (r != 0) {
        xil_printf("Failed to initialize BCH engine (%d)\n", r);
        return XST_FAILURE;
    }

    wfe();

    while (TRUE) {
        dequeue_requests();

        if (send_ipi) {
            ipi_trigger();
            send_ipi = FALSE;
        }

        usleep(1);
    }

    return XST_SUCCESS;
}
