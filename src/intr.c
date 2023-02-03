#include "xparameters.h"
#include "xil_exception.h"
#include "xdebug.h"
#include "xil_io.h"
#include "xscugic.h"

#define INTC_DEVICE_ID XPAR_SCUGIC_SINGLE_DEVICE_ID
#define INTC           XScuGic
#define INTC_HANDLER   XScuGic_InterruptHandler

static INTC Intc; /* Instance of the Interrupt Controller */

int intr_setup(void)
{
    int Status;
    XScuGic_Config* IntcConfig;
    INTC* IntcInstancePtr = &Intc;

    IntcConfig = XScuGic_LookupConfig(INTC_DEVICE_ID);
    if (NULL == IntcConfig) {
        return XST_FAILURE;
    }

    Status = XScuGic_CfgInitialize(IntcInstancePtr, IntcConfig,
                                   IntcConfig->CpuBaseAddress);
    if (Status != XST_SUCCESS) {
        return XST_FAILURE;
    }

    /* Enable interrupts from the hardware */
    Xil_ExceptionInit();
    Xil_ExceptionRegisterHandler(XIL_EXCEPTION_ID_INT,
                                 (Xil_ExceptionHandler)INTC_HANDLER,
                                 (void*)IntcInstancePtr);

    return XST_SUCCESS;
}

int intr_setup_irq(u16 intr_id, int trigger_type, Xil_InterruptHandler handler,
                   void* cb_data)
{
    INTC* IntcInstancePtr = &Intc;

    XScuGic_SetPriorityTriggerType(IntcInstancePtr, intr_id, 0xA0,
                                   trigger_type);
    return XScuGic_Connect(IntcInstancePtr, intr_id, handler, cb_data);
}

void intr_enable(void) { Xil_ExceptionEnable(); }

void intr_disable(void) { Xil_ExceptionDisable(); }

int intr_enable_irq(u16 intr_id)
{
    XScuGic_Enable(&Intc, intr_id);
    return XST_SUCCESS;
}

int intr_disable_irq(u16 intr_id)
{
    XScuGic_Disable(&Intc, intr_id);
    return XST_SUCCESS;
}

void intr_disconnect_irq(u16 intr_id) { XScuGic_Disconnect(&Intc, intr_id); }

unsigned long intr_save(void) { return mfcpsr(); }

void intr_restore(unsigned long flags)
{
    if (flags & XIL_EXCEPTION_IRQ)
        intr_disable();
    else
        intr_enable();
}
