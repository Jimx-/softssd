#ifndef _PCIE_SOFT_INTF_H_
#define _PCIE_SOFT_INTF_H_

#include <stddef.h>

struct pcie_soft_intf {
    void* reg_base;
};

/* Registers */

/* RC DMA */
#define RCDMA_TAG_REG     0x0
#define RCDMA_LENGTH_REG  0x4
#define RCDMA_ADDR_REG    0x8
#define RCDMA_WR_EN_REG   0x10
#define RCDMA_IOC_ACK_REG 0x20

#define RCDMA_IOC_BITMAP_REG 0x8
#define RCDMA_ERR_BITMAP_REG 0x10

/* MSI */
#define MSI_STATUS_REG 0x100

#define MSI_FUNC_ATTR_REG  0x100
#define MSI_INT_VECTOR_REG 0x104

#define MSI_SR_SENT 0x10
#define MSI_SR_FAIL 0x20

void psif_init(struct pcie_soft_intf* psif, void* reg_base);

void psif_rc_setup_buffer(struct pcie_soft_intf* psif, u8 tag, void* addr,
                          size_t len);

u64 psif_get_ioc_bitmap(struct pcie_soft_intf* psif);
u64 psif_get_err_bitmap(struct pcie_soft_intf* psif);
void psif_rc_ack_intr(struct pcie_soft_intf* psif, u64 bitmap);

int psif_send_msi(struct pcie_soft_intf* psif, u16 vector);

#endif
