/* Transaction Scheduling Unit */

#include "xil_types.h"
#include "xil_assert.h"

#include <flash_config.h>
#include <flash.h>
#include "fil.h"

struct txn_queues {
    struct list_head read_queue;
    struct list_head write_queue;
    struct list_head mapping_read_queue;
    struct list_head mapping_write_queue;
    struct list_head gc_read_queue;
    struct list_head gc_write_queue;
    struct list_head gc_erase_queue;
};

static struct txn_queues chip_queues[NR_CHANNELS][CHIPS_PER_CHANNEL];
/* Which chip should be processed next following round-robin order. */
static unsigned int channel_rr_index[NR_CHANNELS];

void tsu_process_task(struct fil_task* txn)
{
    struct txn_queues* chip = &chip_queues[txn->addr.channel][txn->addr.chip];
    struct list_head* queue;

    if (txn->addr.channel >= NR_CHANNELS ||
        txn->addr.chip >= CHIPS_PER_CHANNEL) {
        /* Out-of-range address. */
        notify_task_complete(txn, TRUE);
        return;
    }

    switch (txn->type) {
    case TXN_READ:
        switch (txn->source) {
        case TS_USER_IO:
            queue = &chip->read_queue;
            break;
        case TS_MAPPING:
            queue = &chip->mapping_read_queue;
            break;
        case TS_GC:
            queue = &chip->gc_read_queue;
            break;
        }
        break;
    case TXN_WRITE:
        switch (txn->source) {
        case TS_USER_IO:
            queue = &chip->write_queue;
            break;
        case TS_MAPPING:
            queue = &chip->mapping_write_queue;
            break;
        case TS_GC:
            queue = &chip->gc_write_queue;
            break;
        }
        break;
    case TXN_ERASE:
        queue = &chip->gc_erase_queue;
        break;
    }

    Xil_AssertVoid(queue);

    list_add_tail(&txn->queue, queue);
}

void tsu_init(void)
{
    int i, j;

    for (i = 0; i < NR_CHANNELS; i++) {
        for (j = 0; j < CHIPS_PER_CHANNEL; j++) {
            struct txn_queues* qs = &chip_queues[i][j];
            INIT_LIST_HEAD(&qs->read_queue);
            INIT_LIST_HEAD(&qs->write_queue);
            INIT_LIST_HEAD(&qs->mapping_read_queue);
            INIT_LIST_HEAD(&qs->mapping_write_queue);
            INIT_LIST_HEAD(&qs->gc_read_queue);
            INIT_LIST_HEAD(&qs->gc_write_queue);
            INIT_LIST_HEAD(&qs->gc_erase_queue);
        }
    }
}

static inline int task_ready(struct fil_task* txn) { return TRUE; }

static int dispatch_queue_request(struct list_head* q_prim,
                                  struct list_head* q_sec, enum txn_type type)
{
    struct fil_task* head = NULL;
    unsigned int die;
    unsigned int page;
    struct list_head dispatch_list;
    int max_batch_nr = 1;
    struct fil_task *txn, *tmp;
    uint64_t plane_bitmap = 0;
    int found = 0;

#ifdef ENABLE_MULTIPLANE
    max_batch_nr = PLANES_PER_DIE;
#endif

    list_for_each_entry(txn, q_prim, queue)
    {
        if (!fil_is_die_busy(txn->addr.channel, txn->addr.chip, txn->addr.die,
                             txn->type == TXN_WRITE)) {
            head = txn;
            die = head->addr.die;
            page = head->addr.page;
            break;
        }
    }

    if (!head) return FALSE;

    INIT_LIST_HEAD(&dispatch_list);

    list_for_each_entry_safe(txn, tmp, q_prim, queue)
    {
        if (task_ready(txn) && txn->addr.die == die &&
            !(plane_bitmap & (1 << txn->addr.plane)) &&
            (!plane_bitmap || txn->addr.page == page)) {
            found++;
            plane_bitmap |= 1 << txn->addr.plane;
            list_del(&txn->queue);
            list_add_tail(&txn->queue, &dispatch_list);
        }

        if (found >= max_batch_nr) break;
    }

    if (q_sec && found < max_batch_nr) {
        list_for_each_entry_safe(txn, tmp, q_sec, queue)
        {
            if (task_ready(txn) && txn->addr.die == die &&
                !(plane_bitmap & (1 << txn->addr.plane)) &&
                (!plane_bitmap || txn->addr.page == page)) {
                plane_bitmap |= 1 << txn->addr.plane;
                list_del(&txn->queue);
                list_add_tail(&txn->queue, &dispatch_list);
            }

            if (found >= max_batch_nr) break;
        }
    }

    if (!list_empty(&dispatch_list)) {
        fil_dispatch(&dispatch_list);
        return TRUE;
    }

    return FALSE;
}

static int dispatch_read_request(unsigned int channel, unsigned int chip)
{
    struct list_head *q_prim = NULL, *q_sec = NULL;
    struct txn_queues* queues = &chip_queues[channel][chip];

    if (!list_empty(&queues->mapping_read_queue)) {
        /* Prioritize read txns for mapping entries. */
        q_prim = &queues->mapping_read_queue;

        if (!list_empty(&queues->read_queue))
            q_sec = &queues->read_queue;
        else if (!list_empty(&queues->gc_read_queue))
            q_sec = &queues->gc_read_queue;
    } else {
        if (!list_empty(&queues->read_queue)) {
            q_prim = &queues->read_queue;
            if (!list_empty(&queues->gc_read_queue)) {
                q_sec = &queues->gc_read_queue;
            }
        } else if (!list_empty(&queues->write_queue))
            return FALSE;
        else if (!list_empty(&queues->gc_read_queue))
            q_prim = &queues->gc_read_queue;
        else
            return FALSE;
    }

    return dispatch_queue_request(q_prim, q_sec, TXN_READ);
}

static int dispatch_write_request(unsigned int channel, unsigned int chip)
{
    struct list_head *q_prim = NULL, *q_sec = NULL;
    struct txn_queues* queues = &chip_queues[channel][chip];

    if (!list_empty(&queues->mapping_write_queue)) {
        /* Prioritize write txns for mapping entries. */
        q_prim = &queues->mapping_write_queue;

        if (!list_empty(&queues->write_queue))
            q_sec = &queues->write_queue;
        else if (!list_empty(&queues->gc_write_queue))
            q_sec = &queues->gc_write_queue;
    } else {
        if (!list_empty(&queues->write_queue)) {
            q_prim = &queues->write_queue;
            if (!list_empty(&queues->gc_write_queue)) {
                q_sec = &queues->gc_write_queue;
            }
        } else if (!list_empty(&queues->gc_write_queue))
            q_prim = &queues->gc_write_queue;
        else
            return FALSE;
    }

    return dispatch_queue_request(q_prim, q_sec, TXN_WRITE);
}

static int dispatch_erase_request(unsigned int channel, unsigned int chip)
{
    struct txn_queues* queues = &chip_queues[channel][chip];
    struct list_head* q_prim = &queues->gc_erase_queue;

    if (list_empty(q_prim)) return FALSE;

    return dispatch_queue_request(q_prim, NULL, TXN_ERASE);
}

static void dispatch_request(unsigned int channel, unsigned int chip)
{
    if (dispatch_read_request(channel, chip)) return;
    if (dispatch_write_request(channel, chip)) return;
    dispatch_erase_request(channel, chip);
}

static void tsu_flush_channel(unsigned int channel)
{
    int i;

    for (i = 0; i < CHIPS_PER_CHANNEL; i++) {
        unsigned int chip_id = channel_rr_index[channel];
        dispatch_request(channel, chip_id);
        channel_rr_index[channel] =
            (channel_rr_index[channel] + 1) % CHIPS_PER_CHANNEL;

        if (fil_is_channel_busy(channel)) break;
    }
}

void tsu_flush_queues(void)
{
    int i;

    for (i = 0; i < NR_CHANNELS; i++) {
        if (fil_is_channel_busy(i)) continue;

        tsu_flush_channel(i);
    }
}

void tsu_notify_channel_idle(unsigned int channel)
{
    tsu_flush_channel(channel);
}

void tsu_notify_chip_idle(unsigned int channel, unsigned int chip)
{
    if (fil_is_channel_busy(channel)) return;
    dispatch_request(channel, chip);
}
