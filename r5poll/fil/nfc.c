/* NAND Flash Controller Core */
#include "xdebug.h"
#include "xil_assert.h"
#include <errno.h>
#include <stdlib.h>

#include <utils.h>

#include "flash_config.h"
#include "nfc.h"

#define SR_FAIL   (1 << 0)
#define SR_FAILC  (1 << 1)
#define SR_ARDY   (1 << 5)
#define SR_RDY    (1 << 6)
#define SR_WP     (1 << 7)
#define SR_ALLRDY (SR_ARDY | SR_RDY | SR_WP)

#define pack_address pack_address_micron

static inline u32 nfc_readl(struct nf_controller* nfc, int reg)
{
    return *(volatile u32*)(nfc->base + reg);
}

static inline void nfc_writel(struct nf_controller* nfc, int reg, u32 val)
{
    *(volatile u32*)(nfc->base + reg) = val;
}

static void wait_cmd(struct nf_controller* nfc)
{
    while (!(nfc_readl(nfc, NFC_STATUS_REG) & NFC_SR_IDLE))
        ;
}

static int init_sgdma(struct XAxiDma* axi_dma, u16 dma_id, void* rx_bdbuf,
                      void* tx_bdbuf, size_t bd_size)
{
    XAxiDma_Config* dma_config;
    int status;

    dma_config = XAxiDma_LookupConfig(dma_id);
    if (!dma_config) {
        xil_printf("No config found for %d\r\n", dma_id);
        return XST_FAILURE;
    }

    status = XAxiDma_CfgInitialize(axi_dma, dma_config);
    if (status != XST_SUCCESS) {
        xil_printf("Initialization failed %d\r\n", status);
        return XST_FAILURE;
    }

    if (XAxiDma_HasSg(axi_dma)) {
        xil_printf("Device configured as SG mode \r\n");
        return XST_FAILURE;
    }

    return XST_SUCCESS;
}

int nfc_init(struct nf_controller* nfc, void* base, u16 dma_id,
             unsigned int ecc_size, unsigned int ecc_bytes, void* rx_bdbuf,
             void* tx_bdbuf, size_t bd_size)
{
    int status;

    status = init_sgdma(&nfc->axi_dma, dma_id, rx_bdbuf, tx_bdbuf, bd_size);
    if (status != XST_SUCCESS) {
        xil_printf("DMA initialization failed %d\r\n", status);
        return XST_FAILURE;
    }

    nfc->base = base;

    nfc->step_size = ecc_size;
    nfc->code_size = ecc_bytes;

    while (!((status = nfc_readl(nfc, NFC_STATUS_REG)) & NFC_SR_RDY))
        ;

    return 0;
}

static void pack_address_micron(unsigned int die, unsigned int plane,
                                unsigned int block, unsigned int page,
                                unsigned col, u8* buf)
{
    /* MLC p33 */
    buf[0] = col & 0xff;
    buf[1] = (col >> 8) & 0x7f;
    buf[2] = page & 0xff;
    buf[3] = ((page >> 8) & 1) | ((plane & 1) << 1) | ((block & 0x3f) << 2);
    buf[4] = ((block >> 6) & 0x1f) | ((die & 1) << 5);
}

static void setup_address(struct nf_controller* nfc, unsigned int die,
                          unsigned int plane, unsigned int block,
                          unsigned int page, unsigned col)
{
    u8 buf[5];

    pack_address(die, plane, block, page, col, buf);
    nfc_writel(nfc, NFC_ADDR_L_REG,
               ((u32)buf[3] << 24) | ((u32)buf[2] << 16) | ((u32)buf[1] << 8) |
                   (u32)buf[0]);
    nfc_writel(nfc, NFC_ADDR_H_REG, (u32)buf[4]);
}

void nfc_cmd_reset(struct nf_controller* nfc)
{
    nfc_writel(nfc, NFC_COMMAND_REG, NFC_CMD_RESET);
    wait_cmd(nfc);
}

void nfc_cmd_set_feature(struct nf_controller* nfc, u32 addr, u32 val)
{
    nfc_writel(nfc, NFC_ADDR_H_REG, addr);
    nfc_writel(nfc, NFC_FEAT_REG, val);
    nfc_writel(nfc, NFC_COMMAND_REG, NFC_CMD_SET_FEATURE);
    wait_cmd(nfc);
}

void nfc_cmd_set_feature_lun(struct nf_controller* nfc, unsigned int die,
                             u32 addr, u32 val)
{
    nfc_writel(nfc, NFC_ADDR_L_REG, (die & 0xff) << 24);
    nfc_writel(nfc, NFC_ADDR_H_REG, addr);
    nfc_writel(nfc, NFC_FEAT_REG, val);
    nfc_writel(nfc, NFC_COMMAND_REG, NFC_CMD_SET_FEATURE);
    wait_cmd(nfc);
}

void nfc_cmd_volume_select(struct nf_controller* nfc, unsigned int volume)
{
    nfc_writel(nfc, NFC_ADDR_H_REG, volume);
    nfc_writel(nfc, NFC_COMMAND_REG, NFC_CMD_VOLUME_SELECT);
    wait_cmd(nfc);
}

void nfc_cmd_odt_configure(struct nf_controller* nfc, unsigned int die, u32 val)
{
    nfc_writel(nfc, NFC_ADDR_H_REG, die);
    nfc_writel(nfc, NFC_FEAT_REG, val);
    nfc_writel(nfc, NFC_COMMAND_REG, NFC_CMD_ODT_CONFIGURE);
    wait_cmd(nfc);
}

void nfc_cmd_enable_nvddr2(struct nf_controller* nfc)
{
    nfc_writel(nfc, NFC_COMMAND_REG, NFC_CMD_ENABLE_NVDDR2);
    wait_cmd(nfc);
}

static u8 read_status(struct nf_controller* nfc, unsigned int die,
                      unsigned int plane)
{
    u32 status;

    setup_address(nfc, die, plane, 0, 0, 0);
    nfc_writel(nfc, NFC_COMMAND_REG, NFC_CMD_READ_STATUS);
    wait_cmd(nfc);

    status = nfc_readl(nfc, NFC_STATUS_REG);

    WARN_ONCE(status & NFC_SR_RD_TIMEDOUT, "Read status timed out");

    return status & NFC_SR_STATUS_MASK;
}

u8 nfc_is_ready(struct nf_controller* nfc, unsigned int die, unsigned int plane,
                int* error)
{
    u8 status = read_status(nfc, die, plane);

    if (error) {
        *error = !!(status & (SR_FAIL | SR_FAILC));
    }

    return status == SR_ALLRDY;
}

static int setup_transfer(struct nf_controller* nfc, void* buf, size_t len,
                          void* code_buf, int dir)
{
    int status = 0;

    if (dir == NFC_FROM_NAND) {
        if (!code_buf) {
            code_buf = buf + len;
        }

        len += len / nfc->step_size * nfc->code_size;

        nfc_writel(nfc, NFC_DMA_DATA_ADDR_L_REG, (u32)(UINTPTR)buf);
        nfc_writel(nfc, NFC_DMA_CODE_ADDR_L_REG, (u32)(UINTPTR)code_buf);
        nfc_writel(nfc, NFC_DMA_LENGTH_REG, len);
        nfc_writel(nfc, NFC_DMA_TRIGGER_REG, 1);
    } else {
        status = XAxiDma_SimpleTransfer(&nfc->axi_dma, (UINTPTR)buf, len,
                                        XAXIDMA_DMA_TO_DEVICE);
    }

    return (status != XST_SUCCESS) ? EIO : 0;
}

int nfc_transfer_done(struct nf_controller* nfc, int dir)
{
    int status, dma_busy;

    if (dir == NFC_TO_NAND) {
        dma_busy = XAxiDma_Busy(&nfc->axi_dma, XAXIDMA_DMA_TO_DEVICE);
    } else {
        dma_busy = !(nfc_readl(nfc, NFC_DMA_STATUS_REG) & NFC_DMA_SR_IDLE);
    }

    status = nfc_readl(nfc, NFC_STATUS_REG);

    WARN_ONCE((dir == NFC_FROM_NAND) && (status & NFC_SR_RD_TIMEDOUT),
              "Read transfer timed out");

    return (status & NFC_SR_IDLE) && !dma_busy;
}

int nfc_cmd_read_id(struct nf_controller* nfc, int addr, u8* buf, size_t len)
{
    int r;

    r = setup_transfer(nfc, buf, len, NULL, NFC_FROM_NAND);
    if (r) return r;

    nfc_writel(nfc, NFC_ADDR_H_REG, (u8)addr);
    nfc_writel(nfc, NFC_LENGTH_REG, len);
    nfc_writel(nfc, NFC_COMMAND_REG, NFC_CMD_READ_ID);

    while (!nfc_transfer_done(nfc, NFC_FROM_NAND))
        ;

    Xil_DCacheInvalidateRange((UINTPTR)buf, len);
    return 0;
}

int nfc_cmd_read_page_addr(struct nf_controller* nfc, unsigned int die,
                           unsigned int plane, unsigned int block,
                           unsigned int page, unsigned int col)
{
    setup_address(nfc, die, plane, block, page, col);
    nfc_writel(nfc, NFC_COMMAND_REG, NFC_CMD_READ_MODE);
    wait_cmd(nfc);
    return 0;
}

int nfc_cmd_read_page(struct nf_controller* nfc)
{
    nfc_writel(nfc, NFC_COMMAND_REG, NFC_CMD_READ_PAGE);
    wait_cmd(nfc);
    return 0;
}

int nfc_cmd_read_transfer(struct nf_controller* nfc, unsigned int die,
                          unsigned int plane, u8* buf, size_t len, u8* code_buf)
{
    int r, status;

    status = read_status(nfc, die, plane);
    Xil_AssertNonvoid(status == SR_ALLRDY);

    r = setup_transfer(nfc, buf, len, code_buf, NFC_FROM_NAND);
    if (r) return r;

    /* Data + code length. */
    nfc_writel(nfc, NFC_LENGTH_REG,
               len + len / nfc->step_size * nfc->code_size);
    nfc_writel(nfc, NFC_COMMAND_REG, NFC_CMD_READ_TRANSFER);

    return 0;
}

int nfc_cmd_program_transfer(struct nf_controller* nfc, unsigned int die,
                             unsigned int plane, unsigned int block,
                             unsigned int page, unsigned int col, u8* buf,
                             size_t len)
{
    int r;

    r = setup_transfer(nfc, buf, len, NULL, NFC_TO_NAND);
    if (r) return r;

    setup_address(nfc, die, plane, block, page, col);
    /* Data + code length. */
    nfc_writel(nfc, NFC_LENGTH_REG,
               len + len / nfc->step_size * nfc->code_size);
    nfc_writel(nfc, NFC_COMMAND_REG, NFC_CMD_PROGRAM_MODE);

    return 0;
}

int nfc_cmd_program_page(struct nf_controller* nfc)
{
    nfc_writel(nfc, NFC_COMMAND_REG, NFC_CMD_PROGRAM_PAGE);
    wait_cmd(nfc);
    return 0;
}

int nfc_cmd_erase_block(struct nf_controller* nfc, unsigned int die,
                        unsigned int plane, unsigned int block)
{
    setup_address(nfc, die, plane, block, 0, 0);
    nfc_writel(nfc, NFC_COMMAND_REG, NFC_CMD_ERASE_BLOCK);
    wait_cmd(nfc);
    return 0;
}

int nfc_cmd_get_delay(struct nf_controller* nfc, int select)
{
    int delay_val;

    nfc_writel(nfc, NFC_DELAY_SEL_REG, select);
    nfc_writel(nfc, NFC_COMMAND_REG, NFC_CMD_GET_DELAY);
    wait_cmd(nfc);
    nfc_writel(nfc, NFC_DELAY_SEL_REG, 0);

    delay_val = nfc_readl(nfc, NFC_DELAY_VAL_REG) & 0x1ff;
    return delay_val;
}

static int cmd_set_delay(struct nf_controller* nfc, int select, int delay)
{
    nfc_writel(nfc, NFC_DELAY_SEL_REG, select);
    nfc_writel(nfc, NFC_DELAY_VAL_REG, delay);
    nfc_writel(nfc, NFC_COMMAND_REG, NFC_CMD_SET_DELAY);
    wait_cmd(nfc);
    nfc_writel(nfc, NFC_DELAY_SEL_REG, 0);
    return 0;
}

static int cmd_set_delay_safe(struct nf_controller* nfc, int select, int delay)
{
    int old_val = nfc_cmd_get_delay(nfc, select);
    int cur;

    if (old_val > delay) {
        for (cur = old_val; cur >= delay; cur -= 8) {
            cmd_set_delay(nfc, select, cur);
        }
        if (cur != delay) cmd_set_delay(nfc, select, delay);
    } else if (old_val < delay) {
        for (cur = old_val; cur <= delay; cur += 8) {
            cmd_set_delay(nfc, select, cur);
        }
        if (cur != delay) cmd_set_delay(nfc, select, delay);
    }

    return 0;
}

int nfc_cmd_set_dqs_delay(struct nf_controller* nfc, int delay)
{
    return cmd_set_delay_safe(nfc, NFC_DSR_DQS, delay);
}

int nfc_cmd_set_dq_delay(struct nf_controller* nfc, int bit, int delay)
{
    return cmd_set_delay_safe(nfc, 1 << (1 + bit), delay);
}

int nfc_complete_transfer(struct nf_controller* nfc, int dir, size_t len,
                          u64* err_bitmap)
{
    if (err_bitmap) {
        unsigned int blocks = len / nfc->step_size;
        u32 bitmap_lo, bitmap_hi;
        u64 bitmap;
        int i;

        bitmap_lo = nfc_readl(nfc, NFC_DMA_ERR_BITMAP_L_REG);
        bitmap_hi = nfc_readl(nfc, NFC_DMA_ERR_BITMAP_H_REG);

        bitmap = ((u64)bitmap_hi << 32) | bitmap_lo;

        *err_bitmap = 0;

        for (i = 0; i < blocks; i++) {
            if (bitmap & (1ULL << (blocks - 1 - i))) *err_bitmap |= 1ULL << i;
        }
    }

    return 0;
}
