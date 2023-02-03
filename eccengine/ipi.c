#include "xparameters.h"
#include "xipipsu.h"

static XIpiPsu ipi_psu_inst;

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

    return XST_SUCCESS;
}

int ipi_trigger(void)
{
    return XIpiPsu_TriggerIpi(&ipi_psu_inst,
                              XPAR_XIPIPS_TARGET_PSU_CORTEXA53_0_CH0_MASK);
}
