#include "xparameters.h"
#include "xplatform_info.h"
#include "xuartps.h"
#include "xil_exception.h"
#include "xil_printf.h"

#include <config.h>
#include <utils.h>
#include "proto.h"

#define UART_DEVICE_ID  XPAR_XUARTPS_0_DEVICE_ID
#define UART_INT_IRQ_ID XPAR_XUARTPS_0_INTR

static XUartPs uart_inst;

#define RX_BUFFER_MAX 100
static u8 rx_buffer[RX_BUFFER_MAX];

static void (*recv_data_handler)(const u8*, size_t);

static void uart_handler(void* callback_ref, u32 event, unsigned int event_data)
{
    if (event == XUARTPS_EVENT_RECV_DATA || event == XUARTPS_EVENT_RECV_TOUT) {
        if (recv_data_handler && event_data)
            recv_data_handler(rx_buffer, event_data);

        XUartPs_Recv(&uart_inst, rx_buffer, RX_BUFFER_MAX);
    }
}

int uart_setup(void)
{
    XUartPs_Config* config;
    u32 intr_mask, status;

    config = XUartPs_LookupConfig(UART_DEVICE_ID);
    if (!config) return XST_FAILURE;

    status = XUartPs_CfgInitialize(&uart_inst, config, config->BaseAddress);
    if (status != XST_SUCCESS) {
        return XST_FAILURE;
    }

    status = XUartPs_SelfTest(&uart_inst);
    if (status != XST_SUCCESS) {
        return XST_FAILURE;
    }

    status = intr_setup_irq(UART_INT_IRQ_ID, 0x3,
                            (Xil_InterruptHandler)XUartPs_InterruptHandler,
                            (void*)&uart_inst);
    if (status != XST_SUCCESS) {
        xil_printf("Failed to setup UART IRQ\r\n");
        return XST_FAILURE;
    }

    intr_enable_irq(UART_INT_IRQ_ID);

    XUartPs_SetHandler(&uart_inst, (XUartPs_Handler)uart_handler, &uart_inst);

    intr_mask = XUARTPS_IXR_TOUT | XUARTPS_IXR_PARITY | XUARTPS_IXR_FRAMING |
                XUARTPS_IXR_OVER | XUARTPS_IXR_TXEMPTY | XUARTPS_IXR_RXFULL |
                XUARTPS_IXR_RXOVR;
    XUartPs_SetInterruptMask(&uart_inst, intr_mask);

    XUartPs_SetOperMode(&uart_inst, XUARTPS_OPER_MODE_NORMAL);

    XUartPs_SetRecvTimeout(&uart_inst, 8);

    XUartPs_Recv(&uart_inst, rx_buffer, RX_BUFFER_MAX);

    return 0;
}

void uart_set_recv_data_handler(void (*handler)(const u8*, size_t))
{
    recv_data_handler = handler;
}
