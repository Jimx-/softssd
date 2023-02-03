#include "xparameters.h"
#include "xtmrctr.h"
#include "xil_exception.h"

#include <utils.h>

#define TMRCTR_DEVICE_ID XPAR_TMRCTR_1_DEVICE_ID

#define TMRCTR_FREQ XPAR_AXI_TIMER_1_CLOCK_FREQ_HZ

#define PULSES_PER_US (TMRCTR_FREQ / 1000000UL)
#define PULSES_PER_MS (TMRCTR_FREQ / 1000UL)

#define TIMER_CNTR_0 0
#define TIMER_CNTR_1 1

static XTmrCtr timer_inst;

int timer_setup(void)
{
    int status;

    /* Cascade 64-bit clock source. */

    status = XTmrCtr_Initialize(&timer_inst, TMRCTR_DEVICE_ID);
    if (status != XST_SUCCESS) return XST_FAILURE;

    status = XTmrCtr_SelfTest(&timer_inst, TIMER_CNTR_0);
    if (status != XST_SUCCESS) return XST_FAILURE;

    status = XTmrCtr_SelfTest(&timer_inst, TIMER_CNTR_1);
    if (status != XST_SUCCESS) return XST_FAILURE;

    XTmrCtr_SetResetValue(&timer_inst, TIMER_CNTR_0, 0);
    XTmrCtr_SetResetValue(&timer_inst, TIMER_CNTR_1, 0);

    XTmrCtr_SetOptions(&timer_inst, TIMER_CNTR_0,
                       XTC_AUTO_RELOAD_OPTION | XTC_CASCADE_MODE_OPTION);

    XTmrCtr_Reset(&timer_inst, TIMER_CNTR_0);
    XTmrCtr_Reset(&timer_inst, TIMER_CNTR_1);

    XTmrCtr_Start(&timer_inst, TIMER_CNTR_0);

    return XST_SUCCESS;
}

u64 timer_get_cycles(void)
{
    u32 val0 = XTmrCtr_GetValue(&timer_inst, TIMER_CNTR_0);
    u32 val1 = XTmrCtr_GetValue(&timer_inst, TIMER_CNTR_1);

    return (((u64)val1) << 32) | val0;
}

u32 timer_ms_to_cycles(u32 ms)
{
    if (unlikely(ms >= (UINT32_MAX / PULSES_PER_MS))) return UINT32_MAX;

    return ms * PULSES_PER_MS;
}

u32 timer_us_to_cycles(u32 us)
{
    if (unlikely(us >= (UINT32_MAX / PULSES_PER_US))) return UINT32_MAX;

    return us * PULSES_PER_US;
}

u32 timer_cycles_to_us(u32 cycles)
{
    return (cycles < PULSES_PER_US) ? 1 : (cycles / PULSES_PER_US);
}
