#include "xparameters.h"
#include <stdio.h>
#include <stdlib.h>
#include "xil_printf.h"
#include "xil_mmu.h"
#include "sleep.h"
#include "xil_cache.h"

#include <proto.h>
#include <flash.h>
#include <fil.h>
#include "fil/fil.h"
#include <ringq.h>

#define PS_DDR_LOW_BASE_ADDR 0x1000000UL
#define PS_DDR_LOW_LIMIT     0x80000000UL /* 2G */

static struct ringq fil_ringq;
static int send_ipi;

static int dequeue_requests(void)
{
    uint32_t elem;
    int found = FALSE;
    struct fil_task* task = NULL;

    ringq_read_avail_tail(&fil_ringq);

    while (ringq_get_avail(&fil_ringq, &elem)) {
        found = TRUE;
        task = (struct fil_task*)(uintptr_t)elem;
        Xil_DCacheInvalidateRange((UINTPTR)task, sizeof(*task));

        /* For writes, we also need to invalidate the data buffer. */
        if (task->type == TXN_WRITE)
            Xil_DCacheInvalidateRange((UINTPTR)task->data, task->length);

        tsu_process_task(task);
    }

    return found;
}

static void enqueue_response(struct fil_task* task)
{
    Xil_AssertVoid(task->completed);

    Xil_DCacheFlushRange((UINTPTR)task, sizeof(*task));
    if (task->data) Xil_DCacheFlushRange((UINTPTR)task->data, task->length);

    ringq_add_used(&fil_ringq, (uint32_t)(uintptr_t)task);
    ringq_write_used_tail(&fil_ringq);
    send_ipi = TRUE;
}

static void assert_callback(const char* file, s32 line)
{
    xil_printf("FIL: Assertion failed %s:%d\n", file, line);
}

int main()
{
    void* fil_ringq_buf;

    Xil_AssertSetCallback(assert_callback);

    timer_setup();
    ipi_setup();

    fil_ringq_buf = (void*)(uintptr_t)PS_DDR_LOW_BASE_ADDR;
    Xil_SetTlbAttributes((UINTPTR)fil_ringq_buf,
                         NORM_SHARED_NCACHE | PRIV_RW_USER_NA);

    ringq_init(&fil_ringq, fil_ringq_buf, 1 << 20);

    tsu_init();
    fil_init();

    wfe();

    while (TRUE) {
        int found;

        found = dequeue_requests();

        if (found) tsu_flush_queues();

        fil_tick();

        if (send_ipi) {
            ipi_trigger();
            send_ipi = FALSE;
        }

        usleep(1);
    }

    return 0;
}

void notify_task_complete(struct fil_task* task, int error)
{
    task->status = error ? FTS_ERROR : FTS_OK;
    task->completed = TRUE;

    task->total_xfer_us = timer_cycles_to_us(task->total_xfer_us);
    task->total_exec_us = timer_cycles_to_us(task->total_exec_us);

    enqueue_response(task);
}

void panic(const char* fmt, ...)
{
    char buf[256];
    va_list arg;

    va_start(arg, fmt);
    vsprintf(buf, fmt, arg);
    va_end(arg);

    xil_printf("\nR5 panic: %s\n", buf);

    exit(1);
}
