#include "xil_types.h"
#include "pcie_soft_intf.h"

static inline u32 psif_readl(struct pcie_soft_intf* psif, int reg)
{
    return *(volatile u32*)(psif->reg_base + reg);
}

static inline void psif_writel(struct pcie_soft_intf* psif, int reg, u32 val)
{
    *(volatile u32*)(psif->reg_base + reg) = val;
}

static inline u64 psif_readq(struct pcie_soft_intf* psif, int reg)
{
    u32 lo, hi;

    lo = *(volatile u32*)(psif->reg_base + reg);
    hi = *(volatile u32*)(psif->reg_base + reg + 4);

    return ((u64)hi << 32UL) | lo;
}

static inline void psif_writeq(struct pcie_soft_intf* psif, int reg, u64 val)
{
    *(volatile u32*)(psif->reg_base + reg) = (u32)val;
    *(volatile u32*)(psif->reg_base + reg + 4) = (u32)(val >> 32UL);
}

void psif_init(struct pcie_soft_intf* psif, void* reg_base)
{
    psif->reg_base = reg_base;
}

void psif_rc_setup_buffer(struct pcie_soft_intf* psif, u8 tag, void* addr,
                          size_t len)
{
    psif_writel(psif, RCDMA_TAG_REG, (u32)tag);
    psif_writel(psif, RCDMA_LENGTH_REG, (u32)len);
    psif_writeq(psif, RCDMA_ADDR_REG, (u64)(UINTPTR)addr);
    psif_writel(psif, RCDMA_WR_EN_REG, 1);
}

u64 psif_get_ioc_bitmap(struct pcie_soft_intf* psif)
{
    return psif_readq(psif, RCDMA_IOC_BITMAP_REG);
}

u64 psif_get_err_bitmap(struct pcie_soft_intf* psif)
{
    return psif_readq(psif, RCDMA_ERR_BITMAP_REG);
}

void psif_rc_ack_intr(struct pcie_soft_intf* psif, u64 bitmap)
{
    psif_writeq(psif, RCDMA_IOC_ACK_REG, bitmap);
}

int psif_send_msi(struct pcie_soft_intf* psif, u16 vector)
{
    int old_flags = psif_readl(psif, MSI_STATUS_REG);
    int new_flags, d;

    old_flags = old_flags & (MSI_SR_SENT | MSI_SR_FAIL);
    psif_writel(psif, MSI_FUNC_ATTR_REG, 0);
    psif_writel(psif, MSI_INT_VECTOR_REG, 1 << vector);

    do {
        new_flags = psif_readl(psif, MSI_STATUS_REG);
        new_flags = new_flags & (MSI_SR_SENT | MSI_SR_FAIL);
        d = old_flags ^ new_flags;

        if (d & MSI_SR_SENT) return TRUE;
    } while (!d);

    return FALSE;
}
