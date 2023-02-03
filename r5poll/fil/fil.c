/* Flash Interface Layer */

#include "xdebug.h"
#include "xil_assert.h"
#include "xil_mmu.h"
#include "xparameters.h"
#include "xgpiops.h"
#include <sleep.h>
#include <stddef.h>
#include <errno.h>

#include <config.h>
#include <flash_config.h>
#include <ecc_config.h>
#include "fil.h"
#include "nfc.h"

#include <proto.h>
#include <const.h>
#include <flash.h>
#include <fil.h>

#define BDRING_SIZE (0x2000)

#define INPUT_DELAY_MAX  512
#define INPUT_DELAY_STEP 2

#define TRAINING_BLOCK 12
#define TRAINING_PAGE  1

struct chip_data;
struct channel_data;

struct die_data {
    unsigned int index;
    struct list_head list;
    /* In case of multiplane commands, there can be multiple active transactions
     * on one die. */
    struct list_head active_txns;
    /* Outstanding die list. */
    struct list_head completion;

    struct flash_command cmd_buf;
    /* Active command dispatched to this die (including transfer and execution
     * phase). */
    struct flash_command* active_cmd;
    /* Command being _executed_ on this die. */
    struct flash_command* current_cmd;

    struct chip_data* chip;

    struct fil_task* active_xfer;

    u64 exec_start_cycle;
};

enum chip_status {
    CS_IDLE,
    CS_CMD_DATA_IN,
    CS_WAIT_FOR_DATA_OUT,
    CS_DATA_OUT,
    CS_READING,
    CS_WRITING,
    CS_ERASING,
};

struct chip_data {
    int index;
    int ce_pin;
    enum chip_status status;
    /* Outstanding chip transfer list. */
    struct list_head completion;

    struct channel_data* channel;
    struct die_data dies[DIES_PER_CHIP];
    unsigned int active_dies;

    struct list_head cmd_xfer_queue;
    struct die_data* current_xfer;
    unsigned int nr_waiting_read_xfers;
    u64 last_xfer_start;
};

enum channel_status {
    BUS_IDLE = 0,
    BUS_BUSY,
};

struct channel_data {
    int index;
    enum channel_status status;

    struct nf_controller nfc;

    struct chip_data chips[CHIPS_PER_CHANNEL];

    struct list_head waiting_read_xfer;
};

static struct channel_data channel_data[NR_CHANNELS];

static int ce_pins[] = {CE_PINS};
static int wp_pins[] = {WP_PINS};

static XGpioPs ps_gpio_inst;

static struct nfc_config {
    uintptr_t base_addr;
    int dma_dev_id;
    unsigned int odt_config[CHIPS_PER_CHANNEL];
} nfc_configs[] = {
    {XPAR_ONFI_BCH_PHY_0_BASEADDR, XPAR_AXIDMA_2_DEVICE_ID, {0}},
    {XPAR_ONFI_BCH_PHY_1_BASEADDR, XPAR_AXIDMA_3_DEVICE_ID, {0}},
    {XPAR_ONFI_BCH_PHY_2_BASEADDR, XPAR_AXIDMA_4_DEVICE_ID, {0}},
    {XPAR_ONFI_BCH_PHY_3_BASEADDR, XPAR_AXIDMA_5_DEVICE_ID, {0}},
    {XPAR_ONFI_BCH_PHY_4_BASEADDR, XPAR_AXIDMA_6_DEVICE_ID, {0}},
    {XPAR_ONFI_BCH_PHY_5_BASEADDR, XPAR_AXIDMA_7_DEVICE_ID, {0}},
    {XPAR_ONFI_BCH_PHY_6_BASEADDR, XPAR_AXIDMA_8_DEVICE_ID, {0}},
    {XPAR_ONFI_BCH_PHY_7_BASEADDR, XPAR_AXIDMA_9_DEVICE_ID, {0}},
};

static struct list_head chip_in_transfer_list;
static struct list_head die_command_list;
static struct list_head chip_out_transfer_list;

static void complete_chip_transfer(struct chip_data* chip, u64 timestamp);
static void complete_data_out_transfer(struct chip_data* chip,
                                       struct die_data* die, u64 timestamp);

int fil_is_channel_busy(unsigned int channel)
{
    Xil_AssertNonvoid(channel < NR_CHANNELS);
    return channel_data[channel].status != BUS_IDLE;
}

int fil_is_die_busy(unsigned int channel_nr, unsigned int chip_nr,
                    unsigned int die_nr, int is_program)
{
    struct chip_data* chip;

    Xil_AssertNonvoid(channel_nr < NR_CHANNELS);
    Xil_AssertNonvoid(chip_nr < CHIPS_PER_CHANNEL);
    Xil_AssertNonvoid(die_nr < DIES_PER_CHIP);

    chip = &channel_data[channel_nr].chips[chip_nr];

    if (is_program) {
        /* A PROGRAM-series operation must be issued before the READ-series
         * operation for multi-LUN operations. */
        int i;

        for (i = 0; i < DIES_PER_CHIP; i++) {
            struct die_data* die = &chip->dies[i];

            if (die->active_cmd &&
                (die->active_cmd->cmd_code == FCMD_READ_PAGE ||
                 die->active_cmd->cmd_code == FCMD_READ_PAGE_MULTIPLANE))
                return TRUE;
        }
    }

    return chip->dies[die_nr].active_cmd != NULL;
}

static inline void set_channel_status(struct channel_data* channel,
                                      enum channel_status status)
{
    channel->status = status;
}

static inline void set_chip_status(struct chip_data* chip,
                                   enum chip_status status)
{
    chip->status = status;
}

static inline struct die_data* get_die(unsigned int channel, unsigned int chip,
                                       unsigned int die)
{
    Xil_AssertNonvoid(channel < NR_CHANNELS);
    Xil_AssertNonvoid(chip < CHIPS_PER_CHANNEL);
    Xil_AssertNonvoid(die < DIES_PER_CHIP);
    return &channel_data[channel].chips[chip].dies[die];
}

static inline void write_ce(struct chip_data* chip, int enable)
{
    XGpioPs_WritePin(&ps_gpio_inst, chip->ce_pin, !enable);
}

static inline void select_volume(struct chip_data* chip, int enable)
{
    int i;
    struct channel_data* channel = chip->channel;

    for (i = 0; i < CHIPS_PER_CHANNEL; i++) {
        write_ce(&channel->chips[i], enable);
    }

    if (enable) {
        nfc_cmd_volume_select(&channel->nfc, chip->index);
    }
}

static void init_die(struct die_data* die)
{
    INIT_LIST_HEAD(&die->active_txns);
    die->active_cmd = NULL;
}

static void init_chip(struct chip_data* chip, int ce_pin)
{
    int i;

    chip->ce_pin = ce_pin;
    chip->nr_waiting_read_xfers = 0;
    INIT_LIST_HEAD(&chip->cmd_xfer_queue);

    for (i = 0; i < DIES_PER_CHIP; i++) {
        struct die_data* die = &chip->dies[i];
        die->index = i;
        die->chip = chip;

        init_die(die);
    }
}

static void init_channel(struct channel_data* channel)
{
    channel->status = BUS_IDLE;
    INIT_LIST_HEAD(&channel->waiting_read_xfer);
}

static void alloc_flash_data(void)
{
    int ch_idx, chip_idx;
    int* ce_pp = ce_pins;
    struct channel_data* channel;
    struct chip_data* chip;

    for (ch_idx = 0; ch_idx < NR_CHANNELS; ch_idx++) {
        channel = &channel_data[ch_idx];

        /* Initialize channels first */
        channel->index = ch_idx;
        init_channel(channel);

        /* Match CEs to channels */
        for (chip_idx = 0; chip_idx < CHIPS_PER_CHANNEL; chip_idx++) {
            int ce_pin = *ce_pp++;

            chip = &channel->chips[chip_idx];
            chip->index = chip_idx;
            chip->channel = channel;
            init_chip(chip, ce_pin);
        }
    }
}

static void init_gpio(void)
{
    XGpioPs_Config* gpio_config_ptr;
    int i, status;

    gpio_config_ptr = XGpioPs_LookupConfig(XPAR_PSU_GPIO_0_DEVICE_ID);
    if (gpio_config_ptr == NULL) {
        panic("PS GPIO not found\n\r");
    }

    status = XGpioPs_CfgInitialize(&ps_gpio_inst, gpio_config_ptr,
                                   gpio_config_ptr->BaseAddr);
    if (status != XST_SUCCESS) {
        panic("PS GPIO init failed\n\r");
    }

    for (i = 0; i < sizeof(ce_pins) / sizeof(ce_pins[0]); i++) {
        XGpioPs_SetDirectionPin(&ps_gpio_inst, ce_pins[i], 1);
        XGpioPs_SetOutputEnablePin(&ps_gpio_inst, ce_pins[i], 1);
        XGpioPs_WritePin(&ps_gpio_inst, ce_pins[i], 1);
    }

    for (i = 0; i < sizeof(wp_pins) / sizeof(wp_pins[0]); i++) {
        XGpioPs_SetDirectionPin(&ps_gpio_inst, wp_pins[i], 1);
        XGpioPs_SetOutputEnablePin(&ps_gpio_inst, wp_pins[i], 1);
        XGpioPs_WritePin(&ps_gpio_inst, wp_pins[i], 1);
    }
}

static int erase_block_simple(struct nf_controller* nfc, struct chip_data* chip,
                              unsigned int die, unsigned int plane,
                              unsigned int block)
{
    int ready, error;

    select_volume(chip, TRUE);
    nfc_cmd_erase_block(nfc, die, plane, block);

    do {
        ready = nfc_is_ready(nfc, die, plane, &error);
    } while (!(ready || error));

    select_volume(chip, FALSE);

    return error ? EIO : 0;
}

static int program_page_simple(struct nf_controller* nfc,
                               struct chip_data* chip, unsigned int die,
                               unsigned int plane, unsigned int block,
                               unsigned int page, unsigned int col,
                               const u8* buffer, size_t count)
{
    int ready, error;

    select_volume(chip, TRUE);
    nfc_cmd_program_transfer(nfc, die, plane, block, page, col, buffer, count);
    while (!nfc_transfer_done(nfc, NFC_TO_NAND))
        ;
    nfc_complete_transfer(nfc, NFC_TO_NAND, count, NULL);
    nfc_cmd_program_page(nfc);

    do {
        ready = nfc_is_ready(nfc, die, plane, &error);
    } while (!(ready || error));

    select_volume(chip, FALSE);

    return error ? EIO : 0;
}

static int read_page_simple(struct nf_controller* nfc, struct chip_data* chip,
                            unsigned int die, unsigned int plane,
                            unsigned int block, unsigned int page,
                            unsigned int col, u8* buffer, size_t count,
                            u8* code_buffer, u64* err_bitmap)
{
    int ready, error;

    select_volume(chip, TRUE);
    nfc_cmd_read_page_addr(nfc, die, plane, block, page, col);
    nfc_cmd_read_page(nfc);

    do {
        ready = nfc_is_ready(nfc, die, plane, &error);
    } while (!(ready || error));

    if (error) goto out;
    nfc_cmd_read_transfer(nfc, die, plane, buffer, count, code_buffer);
    while (!nfc_transfer_done(nfc, NFC_FROM_NAND))
        ;
    nfc_complete_transfer(nfc, NFC_FROM_NAND, count, err_bitmap);
    Xil_DCacheInvalidateRange((UINTPTR)buffer, count);

out:
    select_volume(chip, FALSE);

    return error ? EIO : 0;
}

static size_t read_page_test(struct nf_controller* nfc, struct chip_data* chip,
                             unsigned int die, unsigned int plane,
                             unsigned int block, unsigned int page,
                             unsigned int col, u8* buffer, size_t count,
                             u8* code_buffer, u64* err_bitmap,
                             const u8* gt_data, int bit)
{
    size_t err_count = 0;
    int i;

    read_page_simple(nfc, chip, die, plane, block, page, col, buffer, count,
                     code_buffer, err_bitmap);

    for (i = 0; i < count; i++) {
        if (bit == -1) {
            if (buffer[i] != gt_data[i]) err_count++;
        } else {
            if ((buffer[i] & (1 << bit)) != (gt_data[i] & (1 << bit)))
                err_count++;
        }
    }

    return err_count;
}

static int channel_selftest(struct channel_data* channel)
{
    static u8 buffer[2 * FLASH_PG_SIZE + FLASH_PG_OOB_SIZE]
        __attribute__((aligned(0x1000)));
    u8* tx_buffer;
    u8* rx_buffer;
    u8* code_buffer;
    struct nf_controller* nfc = &channel->nfc;
    int i;
    u64 err_bitmap[CHIPS_PER_CHANNEL];
    size_t err_count[CHIPS_PER_CHANNEL];

    tx_buffer = buffer;
    rx_buffer = &buffer[FLASH_PG_SIZE];
    code_buffer = &buffer[2 * FLASH_PG_SIZE];

    for (i = 0; i < FLASH_PG_SIZE; i++)
        tx_buffer[i] = i & 0xff;
    memset(rx_buffer, 0, FLASH_PG_SIZE);
    Xil_DCacheFlushRange((UINTPTR)tx_buffer, FLASH_PG_SIZE);
    Xil_DCacheFlushRange((UINTPTR)rx_buffer, FLASH_PG_SIZE);

    for (i = 0; i < CHIPS_PER_CHANNEL; i++) {
        struct chip_data* chip = &channel->chips[i];

        // Erase block.
        erase_block_simple(nfc, chip, 0, 0, TRAINING_BLOCK);

        // Program page.
        program_page_simple(nfc, chip, 0, 0, TRAINING_BLOCK, TRAINING_PAGE, 0,
                            tx_buffer, FLASH_PG_SIZE);
    }

    int k;
    for (k = 0; k < 10; k++) {
        for (i = 0; i < CHIPS_PER_CHANNEL; i++) {
            struct chip_data* chip = &channel->chips[i];

            err_count[i] = read_page_test(
                nfc, chip, 0, 0, TRAINING_BLOCK, TRAINING_PAGE, 0, rx_buffer,
                FLASH_PG_SIZE, code_buffer, &err_bitmap[i], tx_buffer, -1);
        }

        xil_printf("%d err (%d %d %d %d)\n", k, err_count[0], err_count[1],
                   err_count[2], err_count[3]);
    }

    /* for (i = 0; i < CHIPS_PER_CHANNEL; i++) { */
    /*     struct chip_data* chip = &channel->chips[i]; */
    /*     erase_block_simple(nfc, chip, 0, 0, TRAINING_BLOCK); */
    /* } */

    return TRUE;
}

static void reset_flash(void)
{
    int i, j;

    for (i = 0; i < NR_CHANNELS; i++) {
        struct nf_controller* nfc = &channel_data[i].nfc;

        for (j = 0; j < CHIPS_PER_CHANNEL; j++) {
            struct chip_data* chip = &channel_data[i].chips[j];
            write_ce(chip, TRUE);
            nfc_cmd_reset(nfc);
            write_ce(chip, FALSE);
        }
    }

    usleep(5000);

    /* Appoint volume addresses. */
    for (i = 0; i < NR_CHANNELS; i++) {
        struct nf_controller* nfc = &channel_data[i].nfc;

        for (j = 0; j < CHIPS_PER_CHANNEL; j++) {
            struct chip_data* chip = &channel_data[i].chips[j];
            write_ce(chip, TRUE);
            nfc_cmd_set_feature(nfc, 0x58, chip->index);
            usleep(1);
            write_ce(chip, FALSE);
        }
    }

    usleep(1);

    for (i = 0; i < NR_CHANNELS; i++) {
        struct nf_controller* nfc = &channel_data[i].nfc;
        struct nfc_config* config = &nfc_configs[i];
        struct chip_data* chip;

        for (j = 0; j < CHIPS_PER_CHANNEL; j++) {
            if (config->odt_config[j] == 0) continue;

            chip = &channel_data[i].chips[j];
            write_ce(chip, TRUE);
            nfc_cmd_volume_select(nfc, chip->index);
            usleep(1);
            nfc_cmd_odt_configure(nfc, 0, config->odt_config[j]);
            usleep(1);
            write_ce(chip, FALSE);
        }
    }

    for (i = 0; i < NR_CHANNELS; i++) {
        struct nf_controller* nfc = &channel_data[i].nfc;

        for (j = 0; j < CHIPS_PER_CHANNEL; j++) {
            struct chip_data* chip = &channel_data[i].chips[j];
            int nvddr2_feat = 0x47;

            write_ce(chip, TRUE);
            nfc_cmd_volume_select(nfc, chip->index);
            usleep(1);

            /* Configure RE/DQS differential signaling. */
            nfc_cmd_set_feature(nfc, 2, nvddr2_feat);
            usleep(1);
            /* Configure NV-DDR2 interface. */
            nfc_cmd_set_feature(nfc, 1, 0x27);
            usleep(1);

            write_ce(chip, FALSE);
        }

        nfc_cmd_enable_nvddr2(nfc);
    }

    usleep(10);

#ifdef NFC_SELFTEST
    for (i = 0; i < NR_CHANNELS; i++) {
        xil_printf("Channel %d: ", i);
        channel_selftest(&channel_data[i]);
    }
#endif
}

void fil_init(void)
{
    static u8 bdring_buffer[1 << 20] __attribute__((aligned(1 << 20)));
    int i;

    INIT_LIST_HEAD(&die_command_list);
    INIT_LIST_HEAD(&chip_in_transfer_list);
    INIT_LIST_HEAD(&chip_out_transfer_list);

    alloc_flash_data();

    init_gpio();

    Xil_DCacheInvalidateRange((UINTPTR)bdring_buffer, sizeof(bdring_buffer));
    Xil_SetTlbAttributes((UINTPTR)bdring_buffer,
                         NORM_SHARED_NCACHE | PRIV_RW_USER_NA);

    for (i = 0; i < NR_CHANNELS; i++) {
        struct nf_controller* nfc = &channel_data[i].nfc;
        struct nfc_config* config = &nfc_configs[i];

        nfc_init(
            nfc, (void*)config->base_addr, config->dma_dev_id, BCH_BLOCK_SIZE,
            BCH_CODE_SIZE, bdring_buffer + i * 2 * BDRING_SIZE,
            bdring_buffer + i * 2 * BDRING_SIZE + BDRING_SIZE, BDRING_SIZE);
    }

    reset_flash();
}

static int start_data_out_transfer(struct channel_data* channel)
{
    struct fil_task* txn = NULL;
    struct chip_data* chip;
    struct nf_controller* nfc;
    struct die_data* die;
    unsigned int start_step;
    size_t nr_steps;

    if (channel->status != BUS_IDLE) return FALSE;

    if (!list_empty(&channel->waiting_read_xfer))
        txn = list_entry(channel->waiting_read_xfer.next, struct fil_task,
                         waiting_list);

    if (!txn) return FALSE;

    list_del(&txn->waiting_list);

    chip = &channel->chips[txn->addr.chip];
    nfc = &channel->nfc;
    die = &chip->dies[txn->addr.die];
    Xil_AssertNonvoid(!die->active_xfer);

    set_chip_status(chip, CS_DATA_OUT);
    die->active_xfer = txn;
    set_channel_status(channel, BUS_BUSY);

    /* Only read transactions need data out transfer */
    Xil_AssertNonvoid(txn->type == TXN_READ);

    start_step = txn->offset / nfc->step_size;
    nr_steps = (txn->length + nfc->step_size - 1) / nfc->step_size;

    select_volume(chip, TRUE);

    chip->last_xfer_start = timer_get_cycles();

    if (txn->code_length) {
        nfc_cmd_read_transfer(
            &channel->nfc, txn->addr.die, txn->addr.plane,
            (void*)((uintptr_t)txn->data + start_step * nfc->step_size),
            nr_steps * nfc->step_size,
            (void*)((uintptr_t)txn->code_buf + start_step * nfc->code_size));
    } else {
        nfc_cmd_read_transfer(&channel->nfc, txn->addr.die, txn->addr.plane,
                              (void*)(uintptr_t)txn->data,
                              nr_steps * nfc->step_size, NULL);
    }

    /* Add to outstanding die command list. */
    list_add(&die->completion, &chip_out_transfer_list);

    return TRUE;
}

static int start_die_command(struct chip_data* chip, struct flash_command* cmd)
{
    struct die_data* die = &chip->dies[cmd->addrs[0].die];

    die->exec_start_cycle = timer_get_cycles();
    die->current_cmd = cmd;

    switch (cmd->cmd_code) {
    case FCMD_READ_PAGE:
        nfc_cmd_read_page(&chip->channel->nfc);
        break;
    case FCMD_PROGRAM_PAGE:
        nfc_cmd_program_page(&chip->channel->nfc);
        break;
    default:
        break;
    }

    /* Add to outstanding die command list. */
    list_add_tail(&die->completion, &die_command_list);

    chip->active_dies++;

    return TRUE;
}

static int start_cmd_data_transfer(struct chip_data* chip)
{
    struct die_data* die;
    struct fil_task* head;
    struct nf_controller* nfc;
    unsigned int start_step;
    size_t nr_steps;
    int completed = FALSE;
    u64 now = timer_get_cycles();

    Xil_AssertNonvoid(!chip->current_xfer);
    if (list_empty(&chip->cmd_xfer_queue)) return FALSE;

    nfc = &chip->channel->nfc;
    die = list_entry(chip->cmd_xfer_queue.next, struct die_data, list);
    list_del(&die->list);

    set_chip_status(chip, CS_CMD_DATA_IN);
    chip->current_xfer = die;

    /* TODO: handle multi-plane commands */
    head = list_entry(die->active_txns.next, struct fil_task, queue);

    select_volume(chip, TRUE);

    chip->last_xfer_start = now;

    start_step = head->offset / nfc->step_size;
    nr_steps = (head->length + nfc->step_size - 1) / nfc->step_size;

    switch (die->active_cmd->cmd_code) {
    case FCMD_READ_PAGE:
        /* If code is requested, we re-adjust the offset to point to the
         * real beginning of the ECC block. Otherwise the raw offset is
         * used. */
        nfc_cmd_read_page_addr(
            nfc, head->addr.die, head->addr.plane, head->addr.block,
            head->addr.page,
            (head->code_length > 0)
                ? (start_step * (nfc->step_size + nfc->code_size))
                : head->offset);
        completed = TRUE;
        break;
    case FCMD_PROGRAM_PAGE:
        nfc_cmd_program_transfer(
            nfc, head->addr.die, head->addr.plane, head->addr.block,
            head->addr.page, start_step * (nfc->step_size + nfc->code_size),
            (void*)((uintptr_t)head->data + start_step * nfc->step_size),
            nr_steps * nfc->step_size);
        break;
    case FCMD_ERASE_BLOCK:
        nfc_cmd_erase_block(&chip->channel->nfc, head->addr.die,
                            head->addr.plane, head->addr.block);
        completed = TRUE;
        break;
    default:
        xil_printf("Unsupported flash command (%d)\n",
                   die->active_cmd->cmd_code);
        break;
    }

    if (completed)
        complete_chip_transfer(chip, now);
    else
        list_add(&chip->completion, &chip_in_transfer_list);

    return TRUE;
}

static void complete_chip_transfer(struct chip_data* chip, u64 timestamp)
{
    struct die_data* die = chip->current_xfer;
    struct fil_task* head =
        list_entry(die->active_txns.next, struct fil_task, queue);
    struct channel_data* channel = &channel_data[head->addr.channel];
    struct fil_task* txn;
    unsigned int xfer_time;

    xfer_time = timestamp - chip->last_xfer_start;

    Xil_AssertVoid(!list_empty(&die->active_txns));

    chip->current_xfer = NULL;

    list_for_each_entry(txn, &die->active_txns, queue)
    {
        txn->total_xfer_us += xfer_time;
    }

    start_die_command(chip, die->active_cmd);

    select_volume(chip, FALSE);

    if (!list_empty(&chip->cmd_xfer_queue)) {
        /* Try to interleave command execution & data transfer whenever we
         * can.
         */
        start_cmd_data_transfer(chip);
        return;
    }

    switch (head->type) {
    case TXN_READ:
        set_chip_status(chip, CS_READING);
        break;
    case TXN_WRITE:
        set_chip_status(chip, CS_WRITING);
        break;
    case TXN_ERASE:
        set_chip_status(chip, CS_ERASING);
        break;
    }

    set_channel_status(channel, BUS_IDLE);
    tsu_notify_channel_idle(channel->index);
}

static void complete_die_command(struct chip_data* chip, struct die_data* die,
                                 int error, u64 timestamp)
{
    struct flash_command* cmd = die->current_cmd;
    struct fil_task *txn, *tmp;
    unsigned int exec_cycles;
    int txn_completed = TRUE;

    exec_cycles = timestamp - die->exec_start_cycle;
    chip->active_dies--;
    die->current_cmd = NULL;

    switch (cmd->cmd_code) {
    case FCMD_READ_PAGE:
    case FCMD_READ_PAGE_MULTIPLANE:
        if (!error) {
            if (!chip->active_dies) set_chip_status(chip, CS_WAIT_FOR_DATA_OUT);

            list_for_each_entry(txn, &die->active_txns, queue)
            {
                txn->total_exec_us += exec_cycles;
                chip->nr_waiting_read_xfers++;
                list_add(&txn->waiting_list, &chip->channel->waiting_read_xfer);
            }

            start_data_out_transfer(chip->channel);

            txn_completed = FALSE;
        }
        break;
    default:
        break;
    }

    if (txn_completed) {
        list_for_each_entry_safe(txn, tmp, &die->active_txns, queue)
        {
            txn->total_exec_us += exec_cycles;
            /* In notify_transaction_complete(), the thread that produced
               this txn gets immediately notified and it may release txn
               before we continue to the next txn in the list so
               list_for_each_entry_safe is needed. */
            notify_task_complete(txn, error);
        }
        INIT_LIST_HEAD(&die->active_txns);
        die->active_cmd = NULL;

        if (!chip->active_dies) set_chip_status(chip, CS_IDLE);
    }

    if (chip->channel->status == BUS_IDLE)
        tsu_notify_channel_idle(chip->channel->index);
    if (chip->status == CS_IDLE)
        tsu_notify_chip_idle(chip->channel->index, chip->index);
}

static void complete_data_out_transfer(struct chip_data* chip,
                                       struct die_data* die, u64 timestamp)
{
    struct fil_task* txn = die->active_xfer;
    struct flash_command* cmd = die->active_cmd;
    unsigned int xfer_time;
    uint64_t bitmap;
    int i;

    nfc_complete_transfer(&chip->channel->nfc, NFC_FROM_NAND, txn->length,
                          &bitmap);
    select_volume(chip, FALSE);

    xfer_time = timestamp - chip->last_xfer_start;
    txn->total_xfer_us += xfer_time;

    txn->err_bitmap = bitmap << (txn->offset / chip->channel->nfc.step_size);

    die->active_xfer = NULL;

    for (i = 0; i < cmd->nr_addrs; i++) {
        if (cmd->addrs[i].plane == txn->addr.plane)
            txn->lpa = cmd->metadata[i].lpa;
    }

    list_del(&txn->queue);
    notify_task_complete(txn, FALSE);

    if (list_empty(&die->active_txns)) {
        die->active_cmd = NULL;
    }

    if (!chip->active_dies) {
        if (!--chip->nr_waiting_read_xfers) {
            set_chip_status(chip, CS_IDLE);
        } else {
            set_chip_status(chip, CS_WAIT_FOR_DATA_OUT);
        }
    }

    set_channel_status(chip->channel, BUS_IDLE);
    tsu_notify_channel_idle(chip->channel->index);
}

void fil_dispatch(struct list_head* txn_list)
{
    struct fil_task* head;
    struct fil_task* txn;
    struct channel_data* channel;
    struct chip_data* chip;
    struct die_data* die;
    unsigned int txn_count = 0;

    if (list_empty(txn_list)) return;

    head = list_entry(txn_list->next, struct fil_task, queue);
    channel = &channel_data[head->addr.channel];
    chip = &channel->chips[head->addr.chip];
    die = &chip->dies[head->addr.die];

    Xil_AssertVoid(!die->active_cmd);
    Xil_AssertVoid(channel->status == BUS_IDLE || chip->current_xfer);

    /* Arm die command. */
    die->active_cmd = &die->cmd_buf;
    /* Copy transaction addresses and metadata. */
    list_for_each_entry(txn, txn_list, queue)
    {
        Xil_AssertVoid(txn_count < MAX_CMD_ADDRS);

        struct flash_address* addr = &die->active_cmd->addrs[txn_count];
        struct page_metadata* metadata =
            &die->active_cmd->metadata[txn_count++];
        *addr = txn->addr;
        metadata->lpa = txn->lpa;
    }
    die->active_cmd->nr_addrs = txn_count;

    list_splice_init(txn_list, &die->active_txns);

    /* Set channel busy for command transfer. */
    set_channel_status(channel, BUS_BUSY);

    switch (head->type) {
    case TXN_READ:
        if (txn_count == 1)
            die->active_cmd->cmd_code = FCMD_READ_PAGE;
        else
            die->active_cmd->cmd_code = FCMD_READ_PAGE_MULTIPLANE;
        break;
    case TXN_WRITE:
        if (txn_count == 1)
            die->active_cmd->cmd_code = FCMD_PROGRAM_PAGE;
        else
            die->active_cmd->cmd_code = FCMD_PROGRAM_PAGE_MULTIPLANE;
        break;
    case TXN_ERASE:
        if (txn_count == 1)
            die->active_cmd->cmd_code = FCMD_ERASE_BLOCK;
        else
            die->active_cmd->cmd_code = FCMD_ERASE_BLOCK_MULTIPLANE;
        break;
    }

    list_add_tail(&die->list, &chip->cmd_xfer_queue);
    start_cmd_data_transfer(chip);
}

static void check_completion(void)
{
    struct die_data *die, *tmp_die;
    struct chip_data *chip, *tmp_chip;
    u64 now = timer_get_cycles();
    int i;

    list_for_each_entry_safe(chip, tmp_chip, &chip_in_transfer_list, completion)
    {
        if (nfc_transfer_done(&chip->channel->nfc, NFC_TO_NAND)) {
            list_del(&chip->completion);
            nfc_complete_transfer(&chip->channel->nfc, NFC_TO_NAND, 0, NULL);
            complete_chip_transfer(chip, now);
        }
    }

    list_for_each_entry_safe(die, tmp_die, &die_command_list, completion)
    {
        struct chip_data* chip = die->chip;
        int ready, error = FALSE;

        if (chip->channel->status != BUS_IDLE) continue;

        select_volume(chip, TRUE);
        ready = nfc_is_ready(&chip->channel->nfc, die->index,
                             die->current_cmd->addrs[0].plane, &error);
        select_volume(chip, FALSE);

        if (die->current_cmd->cmd_code == FCMD_READ_PAGE ||
            die->current_cmd->cmd_code == FCMD_READ_PAGE_MULTIPLANE) {
            /* Error bits should be ignore for read commands. */
            error = 0;
        }

        if (ready || error) {
            list_del(&die->completion);
            complete_die_command(die->chip, die, error, now);
        }
    }

    list_for_each_entry_safe(die, tmp_die, &chip_out_transfer_list, completion)
    {
        struct chip_data* chip = die->chip;

        if (nfc_transfer_done(&chip->channel->nfc, NFC_FROM_NAND)) {
            list_del(&die->completion);
            complete_data_out_transfer(die->chip, die, now);
        }
    }

    for (i = 0; i < NR_CHANNELS; i++) {
        struct channel_data* channel = &channel_data[i];

        start_data_out_transfer(channel);
        if (channel->status == BUS_IDLE) tsu_notify_channel_idle(i);
    }
}

void fil_tick(void) { check_completion(); }
