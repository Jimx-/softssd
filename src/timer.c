#include "xparameters.h"
#include "xtmrctr.h"
#include "xil_exception.h"

#include <config.h>
#include <utils.h>
#include <thread.h>
#include "proto.h"

#define TMRCTR_DEVICE_ID    XPAR_TMRCTR_0_DEVICE_ID
#define TMRCTR_INTERRUPT_ID XPAR_FABRIC_TMRCTR_0_VEC_ID

#define TMRCTR_FREQ XPAR_AXI_TIMER_0_CLOCK_FREQ_HZ

#define PULSES_PER_US (TMRCTR_FREQ / 1000000UL)
#define PULSES_PER_MS (TMRCTR_FREQ / 1000UL)

#define TIMER_CNTR_0 0
#define TIMER_CNTR_1 1

static XTmrCtr timer_inst;

static void timer_counter_handler(void* callback_ref, u8 counter_nr)
{
    worker_check_timeout();
}

int timer_setup(void)
{
    int status;
    u32 reset_value;

    /* Use counter 0 as a clock source and counter 1 to generate timer
     * interrupt. */

    status = XTmrCtr_Initialize(&timer_inst, TMRCTR_DEVICE_ID);
    if (status != XST_SUCCESS) return XST_FAILURE;

    status = XTmrCtr_SelfTest(&timer_inst, TIMER_CNTR_0);
    if (status != XST_SUCCESS) return XST_FAILURE;

    status = XTmrCtr_SelfTest(&timer_inst, TIMER_CNTR_1);
    if (status != XST_SUCCESS) return XST_FAILURE;

    status = intr_setup_irq(TMRCTR_INTERRUPT_ID, 0x3,
                            (Xil_InterruptHandler)XTmrCtr_InterruptHandler,
                            (void*)&timer_inst);
    if (status != XST_SUCCESS) {
        xil_printf("Failed to setup timer IRQ\r\n");
        return XST_FAILURE;
    }

    intr_enable_irq(TMRCTR_INTERRUPT_ID);

    XTmrCtr_SetHandler(&timer_inst, timer_counter_handler, &timer_inst);

    XTmrCtr_SetOptions(&timer_inst, TIMER_CNTR_1,
                       XTC_INT_MODE_OPTION | XTC_AUTO_RELOAD_OPTION |
                           XTC_DOWN_COUNT_OPTION);

    reset_value = (1000 / SYSTEM_HZ) * PULSES_PER_MS;
    XTmrCtr_SetResetValue(&timer_inst, TIMER_CNTR_1, reset_value);

    XTmrCtr_Start(&timer_inst, TIMER_CNTR_1);

    XTmrCtr_SetOptions(&timer_inst, TIMER_CNTR_0, XTC_AUTO_RELOAD_OPTION);
    XTmrCtr_SetResetValue(&timer_inst, TIMER_CNTR_0, 0);
    XTmrCtr_Start(&timer_inst, TIMER_CNTR_0);

    return XST_SUCCESS;
}

u32 timer_get_cycles(void)
{
    return XTmrCtr_GetValue(&timer_inst, TIMER_CNTR_0);
}

u32 timer_ms_to_cycles(u32 ms)
{
    if (unlikely(ms >= (UINT32_MAX / PULSES_PER_MS))) return UINT32_MAX;

    return ms * PULSES_PER_MS;
}

u32 timer_cycles_to_us(u32 cycles)
{
    return (cycles < PULSES_PER_US) ? 1 : (cycles / PULSES_PER_US);
}
