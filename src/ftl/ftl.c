#include <types.h>
#include <flash_config.h>
#include <flash.h>
#include <proto.h>
#include <const.h>

#include <histogram.h>

#include <errno.h>

static struct {
    histogram_t read_txns_per_req_hist;
    histogram_t write_txns_per_req_hist;
    histogram_t total_txns_per_req_hist;
    histogram_t ecc_error_blocks_per_req_hist;

    histogram_t read_transfer_time_hist;
    histogram_t write_transfer_time_hist;
    histogram_t read_command_time_hist;
    histogram_t write_command_time_hist;
} stats;

static int segment_user_request(struct user_request* req)
{
    struct flash_transaction *txn, *tmp;
    unsigned count = 0;
    lha_t slba = req->start_lba;
    int r;

    while (count < req->sector_count) {
        unsigned page_offset = slba % SECTORS_PER_FLASH_PG;
        unsigned int txn_size = SECTORS_PER_FLASH_PG - page_offset;
        lpa_t lpa = slba / SECTORS_PER_FLASH_PG;
        page_bitmap_t bitmap;

        if (count + txn_size > req->sector_count)
            txn_size = req->sector_count - count;

        SLABALLOC(txn);
        if (!txn) {
            r = ENOMEM;
            goto cleanup;
        }

        flash_transaction_init(txn);

        bitmap = ~(~0ULL << txn_size);
        bitmap <<= (slba % SECTORS_PER_FLASH_PG);

        txn->req = req;
        txn->type = (req->req_type == IOREQ_WRITE) ? TXN_WRITE : TXN_READ;
        txn->source = TS_USER_IO;
        txn->nsid = req->nsid;
        txn->lpa = lpa;
        txn->ppa = NO_PPA;
        txn->offset = page_offset << SECTOR_SHIFT;
        txn->length = txn_size << SECTOR_SHIFT;
        txn->bitmap = bitmap;
        list_add_tail(&txn->list, &req->txn_list);

        slba += txn_size;
        count += txn_size;
    }

    return 0;

cleanup:
    list_for_each_entry_safe(txn, tmp, &req->txn_list, list)
    {
        list_del(&txn->list);
        SLABFREE(txn);
    }

    return r;
}

static int process_io_request(struct user_request* req)
{
    struct flash_transaction *txn, *tmp;
    int r;

    r = segment_user_request(req);
    if (r != 0) return r;

    r = dc_process_request(req);

    list_for_each_entry_safe(txn, tmp, &req->txn_list, list)
    {
        list_del(&txn->list);
        SLABFREE(txn);
    }

    histogram_record(stats.read_txns_per_req_hist,
                     req->stats.total_flash_read_txns);
    histogram_record(stats.write_txns_per_req_hist,
                     req->stats.total_flash_write_txns);
    histogram_record(stats.total_txns_per_req_hist,
                     req->stats.total_flash_read_txns +
                         req->stats.total_flash_write_txns);

    if (req->stats.ecc_error_blocks)
        histogram_record(stats.ecc_error_blocks_per_req_hist,
                         req->stats.ecc_error_blocks);

    if (req->stats.flash_read_transfer_us > 0) {
        histogram_record(stats.read_transfer_time_hist,
                         req->stats.flash_read_transfer_us);
        histogram_record(stats.read_command_time_hist,
                         req->stats.flash_read_command_us);
    }

    if (req->stats.flash_write_transfer_us) {
        histogram_record(stats.write_transfer_time_hist,
                         req->stats.flash_write_transfer_us);
        histogram_record(stats.write_command_time_hist,
                         req->stats.flash_write_command_us);
    }

    return r;
}

int ftl_process_request(struct user_request* req)
{
    int r;

    switch (req->req_type) {
    case IOREQ_FLUSH:
        dc_flush_ns(req->nsid);
        r = 0;
        break;
    case IOREQ_READ:
    case IOREQ_WRITE:
        r = process_io_request(req);
        break;
    }

    return r;
}

void ftl_shutdown(int abrupt)
{
    /* Alright, just take our time and write everything so that we can still get
     * them next time ... */
    dc_flush_all();
    amu_shutdown();
    bm_shutdown();
}

void ftl_init(void)
{
    int wipe_ssd = FALSE;
    int wipe_mt = FALSE;
    int full_scan = FALSE;

#ifdef WIPE_SSD
    wipe_ssd = TRUE;
    wipe_mt = TRUE;
#endif

#ifdef WIPE_MAPPING_TABLE
    wipe_mt = TRUE;
#endif

#ifdef FULL_BAD_BLOCK_SCAN
    full_scan = TRUE;
    wipe_mt = TRUE;
#endif

    histogram_init(1, UINT64_C(1000), 1, &stats.read_txns_per_req_hist);
    histogram_init(1, UINT64_C(1000), 1, &stats.write_txns_per_req_hist);
    histogram_init(1, UINT64_C(1000), 1, &stats.total_txns_per_req_hist);
    histogram_init(1, UINT64_C(1000), 1, &stats.ecc_error_blocks_per_req_hist);

    histogram_init(1, UINT64_C(1000), 1, &stats.read_transfer_time_hist);
    histogram_init(1, UINT64_C(1000), 1, &stats.write_transfer_time_hist);
    histogram_init(1, UINT64_C(1000), 1, &stats.read_command_time_hist);
    histogram_init(1, UINT64_C(1000), 1, &stats.write_command_time_hist);

    dc_init(CONFIG_DATA_CACHE_CAPACITY);
    bm_init(wipe_ssd, full_scan);
    amu_init(CONFIG_MAPPING_TABLE_CAPACITY, wipe_mt);
}

void ftl_report_stats(void)
{
    xil_printf("Read transactions per request\n");
    xil_printf("===========================================\n");
    histogram_print(stats.read_txns_per_req_hist, 2);
    xil_printf("===========================================\n\n");

    xil_printf("Write transactions per request\n");
    xil_printf("===========================================\n");
    histogram_print(stats.write_txns_per_req_hist, 2);
    xil_printf("============================================\n\n");

    xil_printf("Total transactions per request\n");
    xil_printf("===========================================\n");
    histogram_print(stats.total_txns_per_req_hist, 2);
    xil_printf("============================================\n\n");

    xil_printf("ECC error blocks per request\n");
    xil_printf("===========================================\n");
    histogram_print(stats.ecc_error_blocks_per_req_hist, 2);
    xil_printf("============================================\n\n");

    xil_printf("Read transfer time per request (microseconds)\n");
    xil_printf("===========================================\n");
    histogram_print(stats.read_transfer_time_hist, 2);
    xil_printf("===========================================\n\n");

    xil_printf("Read command time per request (microseconds)\n");
    xil_printf("===========================================\n");
    histogram_print(stats.read_command_time_hist, 2);
    xil_printf("===========================================\n\n");

    xil_printf("Write transfer time per request (microseconds)\n");
    xil_printf("===========================================\n");
    histogram_print(stats.write_transfer_time_hist, 2);
    xil_printf("===========================================\n\n");

    xil_printf("Write command time per request (microseconds)\n");
    xil_printf("===========================================\n");
    histogram_print(stats.write_command_time_hist, 2);
    xil_printf("===========================================\n\n");

    dc_report_stats();
}
