#ifndef _FIL_NFC_H_
#define _FIL_NFC_H_

#include "xaxidma.h"

struct nf_controller {
    void* base;
    XAxiDma axi_dma;

    unsigned int step_size;
    unsigned int code_size;
};

#define NFC_STATUS_REG    0x0
#define NFC_COMMAND_REG   0x4
#define NFC_ADDR_L_REG    0x8
#define NFC_ADDR_H_REG    0xc
#define NFC_FEAT_REG      0x10
#define NFC_LENGTH_REG    0x14
#define NFC_DELAY_VAL_REG 0x18
#define NFC_DELAY_SEL_REG 0x1c

/* DMA registers */
#define NFC_DMA_STATUS_REG       0x40
#define NFC_DMA_ERR_BITMAP_L_REG 0x44
#define NFC_DMA_ERR_BITMAP_H_REG 0x48
#define NFC_DMA_TRIGGER_REG      0x40
#define NFC_DMA_LENGTH_REG       0x44
#define NFC_DMA_DATA_ADDR_L_REG  0x48
#define NFC_DMA_DATA_ADDR_H_REG  0x4c
#define NFC_DMA_CODE_ADDR_L_REG  0x50
#define NFC_DMA_CODE_ADDR_H_REG  0x54

/* Status register */
#define NFC_SR_STATUS_MASK (0xff)
#define NFC_SR_IDLE        (1 << 8)
#define NFC_SR_RDY         (1 << 9)
#define NFC_SR_RIU_VALID   (1 << 10)
#define NFC_SR_RD_TIMEDOUT (1 << 11)

/* Delay select register */
#define NFC_DSR_DQS (1 << 0)
#define NFC_DSR_DQ  (1 << 1)

/* DMA status register */
#define NFC_DMA_SR_IDLE   (1 << 0)
#define NFC_DMA_SR_SLVERR (1 << 1)
#define NFC_DMA_SR_DECERR (1 << 2)

/* Commands */
#define NFC_CMD_RESET           1
#define NFC_CMD_SET_FEATURE     2
#define NFC_CMD_READ_ID         3
#define NFC_CMD_READ_STATUS     4
#define NFC_CMD_ENABLE_NVDDR2   5
#define NFC_CMD_READ_MODE       6
#define NFC_CMD_PROGRAM_MODE    7
#define NFC_CMD_READ_PAGE       8
#define NFC_CMD_PROGRAM_PAGE    9
#define NFC_CMD_READ_TRANSFER   10
#define NFC_CMD_ERASE_BLOCK     11
#define NFC_CMD_GET_DELAY       12
#define NFC_CMD_SET_DELAY       13
#define NFC_CMD_SET_FEATURE_LUN 14
#define NFC_CMD_VOLUME_SELECT   15
#define NFC_CMD_ODT_CONFIGURE   16

/* Transfer direction */
#define NFC_FROM_NAND 0
#define NFC_TO_NAND   1

int nfc_init(struct nf_controller* nfc, void* base, u16 dma_id,
             unsigned int ecc_size, unsigned int ecc_bytes, void* rx_bdbuf,
             void* tx_bdbuf, size_t bd_size);

u8 nfc_is_ready(struct nf_controller* nfc, unsigned int die, unsigned int plane,
                int* error);

void nfc_cmd_reset(struct nf_controller* nfc);
void nfc_cmd_set_feature(struct nf_controller* nfc, u32 addr, u32 val);
void nfc_cmd_enable_nvddr2(struct nf_controller* nfc);

void nfc_cmd_volume_select(struct nf_controller* nfc, unsigned int volume);
void nfc_cmd_odt_configure(struct nf_controller* nfc, unsigned int die,
                           u32 val);

int nfc_cmd_read_page_addr(struct nf_controller* nfc, unsigned int die,
                           unsigned int plane, unsigned int block,
                           unsigned int page, unsigned int col);
int nfc_cmd_read_page(struct nf_controller* nfc);
int nfc_cmd_read_transfer(struct nf_controller* nfc, unsigned int die,
                          unsigned int plane, u8* buf, size_t len,
                          u8* code_buf);

int nfc_cmd_program_transfer(struct nf_controller* nfc, unsigned int die,
                             unsigned int plane, unsigned int block,
                             unsigned int page, unsigned int col, u8* buf,
                             size_t len);
int nfc_cmd_program_page(struct nf_controller* nfc);

int nfc_cmd_erase_block(struct nf_controller* nfc, unsigned int die,
                        unsigned int plane, unsigned int block);

int nfc_transfer_done(struct nf_controller* nfc, int dir);
int nfc_complete_transfer(struct nf_controller* nfc, int dir, size_t len,
                          u64* err_bitmap);

int nfc_cmd_set_dqs_delay(struct nf_controller* nfc, int delay);
int nfc_cmd_set_dq_delay(struct nf_controller* nfc, int bit, int delay);

#endif
