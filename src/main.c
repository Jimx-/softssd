/***************************** Include Files *********************************/
#include "xparameters.h"
#include "xil_exception.h"
#include "xdebug.h"
#include "xil_io.h"
#include "xil_mmu.h"
#include "xil_cache.h"
#include "ff.h"
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <errno.h>

#include <config.h>
#include <const.h>
#include <proto.h>
#include <thread.h>
#include <libcoro.h>
#include <fil.h>
#include <ecc.h>
#include <ringq.h>
#include <utils.h>

#define PS_DDR_LOW_BASE_ADDR  0x1000000UL
#define PS_DDR_LOW_LIMIT      0x80000000UL /* 2G */
#define PS_DDR_HIGH_BASE_ADDR ((unsigned long)XPAR_PSU_DDR_1_S_AXI_BASEADDR)
#define PS_DDR_HIGH_LIMIT \
    ((unsigned long)XPAR_PSU_DDR_1_S_AXI_HIGHADDR + 1) /* 2G */
#define PL_DDR_BASE_ADDR XPAR_DDR4_0_BASEADDR
#define PL_DDR_LIMIT     (XPAR_DDR4_0_HIGHADDR + 1)

static FATFS fatfs;

static struct ringq fil_ringq;
static struct ringq ecc_ringq;

static void assert_callback(const char* file, s32 line)
{
    xil_printf("FTL: Assertion failed %s:%d\n", file, line);
}

static void enqueue_fil_task(struct fil_task* task)
{
    if (task->data) Xil_DCacheFlushRange((UINTPTR)task->data, task->length);
    if (task->code_buf && task->code_length)
        Xil_DCacheFlushRange((UINTPTR)task->code_buf, task->code_length);
    Xil_DCacheFlushRange((UINTPTR)task, sizeof(*task));

    ringq_add_avail(&fil_ringq, (uint32_t)(uintptr_t)task);
    ringq_write_avail_tail(&fil_ringq);
}

static void handle_fil_response(struct fil_task* resp)
{
    struct worker_thread* worker = (struct worker_thread*)resp->opaque;
    worker_wake(worker, WT_BLOCKED_ON_FIL);
}

static void poll_fil_responses(void)
{
    uint32_t elem;
    struct fil_task* task = NULL;

    ringq_read_used_tail(&fil_ringq);

    while (ringq_get_used(&fil_ringq, &elem)) {
        task = (struct fil_task*)(uintptr_t)elem;
        if (unlikely(!task->completed))
            Xil_DCacheInvalidateRange((UINTPTR)task, sizeof(*task));
        Xil_AssertVoid(task->completed);
        handle_fil_response(task);
    }
}

static void enqueue_ecc_task(struct ecc_task* task)
{
    Xil_DCacheFlushRange((UINTPTR)task->data, task->length);
    Xil_DCacheFlushRange((UINTPTR)task->code, task->code_length);
    Xil_DCacheFlushRange((UINTPTR)task, sizeof(*task));

    ringq_add_avail(&ecc_ringq, (uint32_t)(uintptr_t)task);
    ringq_write_avail_tail(&ecc_ringq);
}

static void handle_ecc_response(struct ecc_task* resp)
{
    struct worker_thread* worker = (struct worker_thread*)resp->opaque;
    worker_wake(worker, WT_BLOCKED_ON_ECC);
}

static void poll_ecc_responses(void)
{
    uint32_t elem;
    struct ecc_task* task = NULL;

    ringq_read_used_tail(&ecc_ringq);

    while (ringq_get_used(&ecc_ringq, &elem)) {
        task = (struct ecc_task*)(uintptr_t)elem;
        if (unlikely(!task->completed))
            Xil_DCacheInvalidateRange((UINTPTR)task, sizeof(*task));
        Xil_AssertVoid(task->completed);
        handle_ecc_response(task);
    }
}

#ifdef FORMAT_EMMC
static int format_emmc(void)
{
    static BYTE work[FF_MAX_SS];
    return f_mkfs("", FM_FAT32, 0, work, sizeof work);
}
#endif

static inline int init_emmc(void) { return f_mount(&fatfs, "", 0); }

int main(void)
{
    int status;
    void *fil_ringq_buf, *ecc_ringq_buf;

    Xil_AssertSetCallback(assert_callback);

    mem_init(MEMZONE_PS_DDR_LOW, PS_DDR_LOW_BASE_ADDR,
             PS_DDR_LOW_LIMIT - PS_DDR_LOW_BASE_ADDR);
    mem_init(MEMZONE_PS_DDR_HIGH, PS_DDR_HIGH_BASE_ADDR,
             PS_DDR_HIGH_LIMIT - PS_DDR_HIGH_BASE_ADDR);
    mem_init(MEMZONE_PL_DDR, PL_DDR_BASE_ADDR, PL_DDR_LIMIT - PL_DDR_BASE_ADDR);

    /* Initialize the control structure of FIL. */
    fil_ringq_buf = alloc_mem(1 << 20, 1 << 20, ZONE_PS_DDR_LOW); /* 1M */
    Xil_AssertNonvoid(fil_ringq_buf == (void*)PS_DDR_LOW_BASE_ADDR);
    Xil_SetTlbAttributes((UINTPTR)fil_ringq_buf,
                         NORM_NONCACHE); /* Uncachable */
    ringq_init(&fil_ringq, fil_ringq_buf, 1 << 20);
    ringq_write_avail_tail(&fil_ringq);
    ringq_write_used_tail(&fil_ringq);

    ecc_ringq_buf = alloc_mem(1 << 20, 1 << 20, ZONE_PS_DDR_LOW); /* 1M */
    Xil_AssertNonvoid(ecc_ringq_buf ==
                      (void*)(PS_DDR_LOW_BASE_ADDR + (1 << 20)));
    Xil_SetTlbAttributes((UINTPTR)ecc_ringq_buf,
                         NORM_NONCACHE); /* Uncachable */
    ringq_init(&ecc_ringq, ecc_ringq_buf, 1 << 20);
    ringq_write_avail_tail(&ecc_ringq);
    ringq_write_used_tail(&ecc_ringq);

    /* Allow FIL to run. */
    sev();
    wfe();

#ifdef FORMAT_EMMC
    xil_printf("Formatting EMMC ...\n");
    format_emmc();
#endif

    status = init_emmc();
    if (status != 0) {
        xil_printf("Failed to initialize EMMC\n");
        return XST_FAILURE;
    }

    tls_init();
    slabs_init();

    status = intr_setup();
    if (status != XST_SUCCESS) {
        xil_printf("Failed to setup interrupt\n");
        return XST_FAILURE;
    }

    status = ipi_setup();
    if (status != XST_SUCCESS) {
        xil_printf("Failed to setup IPI\n");
        return XST_FAILURE;
    }

    status = timer_setup();
    if (status != XST_SUCCESS) {
        xil_printf("Failed to setup timer\n");
        return XST_FAILURE;
    }

    status = uart_setup();
    if (status != XST_SUCCESS) {
        xil_printf("Failed to setup UART\n");
        return XST_FAILURE;
    }

    status = pcie_setup();
    if (status != XST_SUCCESS) {
        xil_printf("Failed to setup PCIe\n");
        return XST_FAILURE;
    }

    dbgcon_setup();

    nvme_init();

    coro_init();
    worker_init(ftl_init);

    xil_printf("\r\n--- Entering nvme main() plddr --- \r\n");

    for (;;) {
        intr_enable();
        wfe();
        intr_disable();

        /* ipi_clear_status(); */

        poll_fil_responses();
        poll_ecc_responses();

        worker_yield();
    }

    pcie_stop();
    xil_printf("--- Exiting main() --- \r\n");

    return XST_SUCCESS;
}

void panic(const char* fmt, ...)
{
    char buf[256];
    va_list arg;

    va_start(arg, fmt);
    vsprintf(buf, fmt, arg);
    va_end(arg);

    xil_printf("\nKernel panic: %s\n", buf);

    exit(1);
}

int submit_flash_transaction(struct flash_transaction* txn)
{
    u8 task_buf[64 + sizeof(struct fil_task)];
    struct fil_task* task =
        (struct fil_task*)((uintptr_t)&task_buf[63] & ~0x3f);
    int r;

    memset(task, 0, sizeof(*task));
    task->addr = txn->addr;
    task->source = txn->source;
    task->type = txn->type;
    task->data = (uint64_t)txn->data;
    task->offset = txn->offset;
    task->length = txn->length;
    task->status = 0;
    task->completed = FALSE;
    task->opaque = (uint64_t)worker_self();
    task->err_bitmap = 0;

    task->code_buf = (uint64_t)txn->code_buf;
    task->code_length = txn->code_length;

    enqueue_fil_task(task);

    r = worker_wait_timeout(WT_BLOCKED_ON_FIL, 3000);
    if (r) return r;

    if (task->status) {
        xil_printf("!!!!! Error t%d s%d ch%d w%d d%d pl%d b%d p%d\n", txn->type,
                   txn->source, task->addr.channel, task->addr.chip,
                   task->addr.die, task->addr.plane, task->addr.block,
                   task->addr.page);
    }

    txn->total_xfer_us = task->total_xfer_us;
    txn->total_exec_us = task->total_exec_us;
    txn->err_bitmap = task->err_bitmap;
    return task->status == FTS_ERROR ? EIO : 0;
}

static int submit_ecc_task(int type, u8* data, size_t data_length, u8* code,
                           size_t code_length, size_t offset,
                           uint64_t err_bitmap)
{
    u8 task_buf[64 + sizeof(struct ecc_task)];
    struct ecc_task* task =
        (struct ecc_task*)((uintptr_t)&task_buf[63] & ~0x3f);
    int r;

    memset(task, 0, sizeof(*task));
    task->type = type;
    task->data = (uint64_t)data;
    task->offset = (uint32_t)offset;
    task->length = (uint32_t)data_length;
    task->code = (uint64_t)code;
    task->code_length = (uint32_t)code_length;
    task->err_bitmap = err_bitmap;
    task->opaque = (uint64_t)worker_self();

    enqueue_ecc_task(task);

    r = worker_wait_timeout(WT_BLOCKED_ON_ECC, 3000);
    if (r) return r;

    switch (task->status) {
    case ETS_OK:
        r = task->code_length;
        break;
    case ETS_NOSPC:
        r = -ENOMEM;
        break;
    case ETS_DEC_ERROR:
        r = -EBADMSG;
        break;
    }

    return r;
}

int ecc_calculate(const u8* data, size_t data_length, u8* code,
                  size_t code_length, size_t offset)
{
    return submit_ecc_task(ECT_CALC, (u8*)data, data_length, code, code_length,
                           offset, 0);
}

int ecc_correct(u8* data, size_t data_length, const u8* code,
                size_t code_length, uint64_t err_bitmap)
{
    return submit_ecc_task(ECT_CORRECT, data, data_length, (u8*)code,
                           code_length, 0, err_bitmap);
}
