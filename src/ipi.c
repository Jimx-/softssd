#include "xparameters.h"
#include "xipipsu.h"

#include <proto.h>
#include <utils.h>

static XIpiPsu ipi_psu_inst;

static void ipi_handler(void* callback)
{
    XIpiPsu* inst_ptr = (XIpiPsu*)callback;
    u32 src_mask;

    src_mask = XIpiPsu_GetInterruptStatus(inst_ptr);
    XIpiPsu_ClearInterruptStatus(inst_ptr, src_mask);
}

int ipi_setup(void)
{
    int status;
    XIpiPsu_Config* config;

    config = XIpiPsu_LookupConfig(XPAR_XIPIPSU_0_DEVICE_ID);
    if (config == NULL) {
        xil_printf("IPI PSU not found\r\n");
        return XST_FAILURE;
    }

    status = XIpiPsu_CfgInitialize(&ipi_psu_inst, config, config->BaseAddress);
    if (status != XST_SUCCESS) {
        xil_printf("IPI config failed\r\n");
        return XST_FAILURE;
    }

    status = intr_setup_irq(XPAR_XIPIPSU_0_INT_ID, 0x1,
                            (Xil_InterruptHandler)ipi_handler, &ipi_psu_inst);
    if (status != XST_SUCCESS) {
        xil_printf("Failed to set up IPI IRQ\r\n");
        return XST_FAILURE;
    }

    intr_enable_irq(XPAR_XIPIPSU_0_INT_ID);

    /* Enable IPI from RPUs. */
    XIpiPsu_InterruptEnable(&ipi_psu_inst, XIPIPSU_ALL_MASK);
    XIpiPsu_ClearInterruptStatus(&ipi_psu_inst, XIPIPSU_ALL_MASK);

    return XST_SUCCESS;
}

void ipi_clear_status(void)
{
    XIpiPsu_ClearInterruptStatus(&ipi_psu_inst, XIPIPSU_ALL_MASK);
}
