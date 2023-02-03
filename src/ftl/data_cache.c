#include <config.h>
#include <types.h>
#include <avl.h>
#include <proto.h>
#include <iov_iter.h>
#include <tls.h>
#include <flash.h>
#include <flash_config.h>
#include <const.h>
#include <utils.h>

#include <errno.h>

static DEFINE_TLS(struct iovec, req_iovecs[1 << MAX_DATA_TRANSFER_SIZE]);

struct cache_stats {
    size_t total_read_hits;
    size_t total_read_misses;
};

enum cache_entry_status {
    CES_EMPTY,
    CES_CLEAN,
    CES_DIRTY,
};

struct cache_entry {
    lpa_t lpa;
    unsigned int nsid;
    page_bitmap_t bitmap;
    enum cache_entry_status status;
    struct avl_node avl;
    struct list_head lru;
    mutex_t mutex;
    void* data;
};

struct data_cache {
    size_t capacity_pages;
    size_t nr_pages;
    unsigned int nsid;
    struct avl_root root;
    struct list_head lru_list;
    struct cache_stats stats;
};

static struct data_cache g_cache;

static inline struct data_cache* get_cache_for_ns(unsigned int nsid)
{
    /* TODO: multiple namespaces */
    Xil_AssertNonvoid(nsid == 1);
    return &g_cache;
}

static inline struct data_cache*
get_cache_for_txn(struct flash_transaction* txn)
{
    return get_cache_for_ns(txn->nsid);
}

static int cache_key_node_comp(void* key, struct avl_node* node)
{
    struct cache_entry* r1 = (struct cache_entry*)key;
    struct cache_entry* r2 = avl_entry(node, struct cache_entry, avl);

    if (r1->lpa < r2->lpa)
        return -1;
    else if (r1->lpa > r2->lpa)
        return 1;
    return 0;
}

static int cache_node_node_comp(struct avl_node* node1, struct avl_node* node2)
{
    struct cache_entry* r1 = avl_entry(node1, struct cache_entry, avl);
    struct cache_entry* r2 = avl_entry(node2, struct cache_entry, avl);

    if (r1->lpa < r2->lpa)
        return -1;
    else if (r1->lpa > r2->lpa)
        return 1;
    return 0;
}

static inline void cache_avl_start_iter(struct data_cache* cache,
                                        struct avl_iter* iter, void* key,
                                        int flags)
{
    avl_start_iter(&cache->root, iter, key, flags);
}

static inline struct cache_entry* cache_avl_get_iter(struct avl_iter* iter)
{
    struct avl_node* node = avl_get_iter(iter);
    if (!node) return NULL;
    return avl_entry(node, struct cache_entry, avl);
}

static void cache_init(struct data_cache* cache, unsigned int nsid,
                       size_t capacity_pages)
{
    cache->capacity_pages = capacity_pages;
    cache->nr_pages = 0;
    cache->nsid = nsid;
    INIT_LIST_HEAD(&cache->lru_list);
    INIT_AVL_ROOT(&cache->root, cache_key_node_comp, cache_node_node_comp);
    memset(&cache->stats, 0, sizeof(cache->stats));
}

static struct cache_entry* cache_find(struct data_cache* cache, lpa_t lpa)
{
    struct avl_node* node = cache->root.node;
    struct cache_entry* entry = NULL;

    while (node) {
        entry = avl_entry(node, struct cache_entry, avl);

        if (entry->lpa == lpa) {
            return entry;
        } else if (lpa < entry->lpa)
            node = node->left;
        else if (lpa > entry->lpa)
            node = node->right;
    }

    return NULL;
}

static void cache_touch_lru(struct data_cache* cache, struct cache_entry* entry)
{
    list_del(&entry->lru);
    list_add(&entry->lru, &cache->lru_list);
}

static struct cache_entry* cache_get(struct data_cache* cache, lpa_t lpa)
{
    struct cache_entry* entry = cache_find(cache, lpa);
    if (!entry) return NULL;

    cache_touch_lru(cache, entry);
    return entry;
}

static inline void add_entry(struct data_cache* cache,
                             struct cache_entry* entry)
{
    avl_insert(&entry->avl, &cache->root);
    cache->nr_pages++;
}

static inline void add_lru(struct data_cache* cache, struct cache_entry* entry)
{
    list_add(&entry->lru, &cache->lru_list);
}

static inline void remove_lru(struct cache_entry* entry)
{
    list_del(&entry->lru);
}

static int cache_add(struct data_cache* cache, lpa_t lpa, page_bitmap_t bitmap,
                     struct cache_entry** entrypp)
{
    struct cache_entry* entry;

    if (cache->nr_pages >= cache->capacity_pages) return ENOSPC;

    SLABALLOC(entry);
    if (!entry) return ENOMEM;

    memset(entry, 0, sizeof(*entry));
    entry->lpa = lpa;
    entry->bitmap = bitmap;
    entry->status = CES_DIRTY;
    INIT_LIST_HEAD(&entry->lru);
    mutex_init(&entry->mutex, NULL);

    /* Allocate data buffer for the entry. Prefer PL DDR. */
    entry->data = alloc_pages(FLASH_PG_SIZE >> PG_SHIFT, ZONE_PL_DDR);
    if (!entry->data) {
        SLABFREE(entry);
        return ENOMEM;
    }

    add_entry(cache, entry);
    *entrypp = entry;

    return 0;
}

static int cache_evict_entry(struct data_cache* cache,
                             struct cache_entry** entrypp)
{
    struct cache_entry* entry;

    if (list_empty(&cache->lru_list)) return ENOMEM;

    entry = list_entry(cache->lru_list.prev, struct cache_entry, lru);

    list_del(&entry->lru);
    avl_erase(&entry->avl, &cache->root);
    cache->nr_pages--;

    *entrypp = entry;
    return 0;
}

static int generate_writeback(struct flash_transaction* txn,
                              struct data_cache* cache,
                              struct cache_entry* entry)
{
    void* data;
    unsigned long i;

    /* We cannot use entry->data directly because it is allocated from PL DDR
     * but FIL only has access to PS low DDR. */
    data = alloc_pages(FLASH_PG_BUFFER_SIZE >> PG_SHIFT, ZONE_PS_DDR_LOW);
    if (!data) return ENOMEM;

    for (i = 0; i < (FLASH_PG_SIZE >> SECTOR_SHIFT); i++) {
        if (entry->bitmap & (1UL << i)) {
            memcpy(data + (i << SECTOR_SHIFT),
                   entry->data + (i << SECTOR_SHIFT), SECTOR_SIZE);
        }
    }

    flash_transaction_init(txn);
    txn->type = TXN_WRITE;
    txn->source = TS_USER_IO;
    txn->nsid = cache->nsid;
    txn->lpa = entry->lpa;
    txn->ppa = NO_PPA;
    txn->data = data;
    txn->offset = 0;
    txn->length = FLASH_PG_SIZE;
    txn->bitmap = entry->bitmap;

    return 0;
}

static int write_buffers(struct user_request* req)
{
    /* Write a user request to the data cache. */
    struct flash_transaction *txn, *tmp;
    struct iovec* iov = get_local_var(req_iovecs);
    struct iov_iter iter;
    size_t count;
    struct list_head wb_txns;
    int i, r;

    INIT_LIST_HEAD(&wb_txns);

    i = 0;
    count = 0;
    /* Setup IO vectors for DMA. */
    list_for_each_entry(txn, &req->txn_list, list)
    {
        struct data_cache* cache = get_cache_for_txn(txn);
        struct cache_entry* entry = cache_find(cache, txn->lpa);

        if (i >= (1 << MAX_DATA_TRANSFER_SIZE)) {
            r = ENOMEM;
            goto cleanup;
        }

        if (entry) {
            /* If the entry already exists then just lock it. */
            remove_lru(entry);
            mutex_lock(&entry->mutex);
            entry->bitmap |= txn->bitmap;
            entry->status = CES_DIRTY;
        } else {
            /* Try to add a new entry. */
            r = cache_add(cache, txn->lpa, txn->bitmap, &entry);

            if (r == ENOSPC) {
                /* Cache is full. */
                r = cache_evict_entry(cache, &entry);
                if (r != 0) goto cleanup;

                if (entry->status == CES_DIRTY) {
                    /* Cache writeback. */
                    struct flash_transaction* wb_txn;

                    SLABALLOC(wb_txn);
                    if (!wb_txn) {
                        r = ENOMEM;
                        goto cleanup;
                    }

                    r = generate_writeback(wb_txn, cache, entry);
                    if (r) {
                        SLABFREE(wb_txn);
                        goto cleanup;
                    }

                    wb_txn->req = txn->req; /* For accounting */
                    list_add_tail(&wb_txn->list, &wb_txns);
                }

                /* XXX: maybe reset the status after the page is written (i.e.
                 * after amu_dispatch) for better consistency. */
                entry->status = CES_DIRTY;
                entry->lpa = txn->lpa;
                entry->bitmap = txn->bitmap;
                add_entry(cache, entry);

                mutex_lock(&entry->mutex);
            } else if (r != 0) {
                /* Error. */
                goto cleanup;
            } else {
                /* Ok. */
                mutex_lock(&entry->mutex);
            }
        }

        txn->opaque = entry;

        iov->iov_base = entry->data + txn->offset;
        iov->iov_len = txn->length;

        iov++;
        i++;
        count += txn->length;
    }

    /* Dispatch writebacks first. */
    if (!list_empty(&wb_txns)) {
        r = amu_dispatch(&wb_txns);
        if (r) goto cleanup;
    }

    /* At this point, all cache entries for the request are allocated,
     * locked and detached from LRU so that they will not be evicted until we
     * finish. */
    iov_iter_init(&iter, get_local_var(req_iovecs), i, count);

    /* Read data into data buffers. */
    r = nvme_dma_read(req, &iter, count);

cleanup:
    list_for_each_entry_safe(txn, tmp, &wb_txns, list)
    {
        list_del(&txn->list);
        if (txn->data) free_mem(txn->data, FLASH_PG_BUFFER_SIZE);
        SLABFREE(txn);
    }

    list_for_each_entry(txn, &req->txn_list, list)
    {
        struct cache_entry* entry = (struct cache_entry*)txn->opaque;
        struct data_cache* cache = get_cache_for_txn(txn);

        /* Reset entry status on failure. */
        if (r != 0) {
            entry->status = CES_EMPTY;
            avl_erase(&entry->avl, &cache->root);
        }

        mutex_unlock(&entry->mutex);
        add_lru(cache, entry);
    }

    return r;
}

static int handle_cached_read(struct user_request* req)
{
    struct flash_transaction *txn, *tmp;
    struct iovec* iov = get_local_var(req_iovecs);
    struct iov_iter iter;
    struct list_head hit_list;
    size_t count;
    int i, r;

    /* All flash transactions with a cache hit are moved to this list. */
    INIT_LIST_HEAD(&hit_list);

    i = 0;
    count = 0;
    /* Setup IO vectors for DMA. */
    list_for_each_entry_safe(txn, tmp, &req->txn_list, list)
    {
        struct data_cache* cache = get_cache_for_txn(txn);
        struct cache_entry* entry = cache_find(cache, txn->lpa);
        page_bitmap_t avail_sectors = 0;

        if (entry) avail_sectors = entry->bitmap & txn->bitmap;

        if (entry && avail_sectors == txn->bitmap) {
            /* Cache hit and all requested sectors are present. */
            remove_lru(entry);
            mutex_lock(&entry->mutex);
            txn->data = entry->data;
            txn->opaque = entry;

            list_del(&txn->list);
            list_add(&txn->list, &hit_list);

            cache->stats.total_read_hits++;
        } else {
            /* Cache miss or some requested sectors are not present. In either
             * case, we need to do a flash read to get the missing sectors. */

            /* Allocate data buffer from PS DDR low because we need to read NAND
             * data. */
            txn->data =
                alloc_pages(FLASH_PG_BUFFER_SIZE >> PG_SHIFT, ZONE_PS_DDR_LOW);
            if (txn->data == NULL) {
                r = ENOMEM;
                goto cleanup;
            }

            txn->bitmap &= ~avail_sectors;
            txn->opaque = NULL;

            if (avail_sectors) {
                /* If there are available sectors we need to overlay
                 * cached data on top of the data read from NAND
                 * flash later. */
                remove_lru(entry);
                mutex_lock(&entry->mutex);
                txn->opaque = entry;
            } else if (entry) {
                cache_touch_lru(cache, entry);
            }

            cache->stats.total_read_misses++;
        }

        iov->iov_base = txn->data + txn->offset;
        iov->iov_len = txn->length;

        iov++;
        i++;
        count += txn->length;
    }

    /* Read data from NAND flash before we overlay the cached sectors. */
    r = amu_dispatch(&req->txn_list);
    if (r) goto cleanup;

    list_for_each_entry(txn, &req->txn_list, list)
    {
        /* Copy usable sectors from cached page. */
        struct cache_entry* entry = (struct cache_entry*)txn->opaque;
        struct data_cache* cache = get_cache_for_txn(txn);
        int i;

        if (!entry) continue;

        for (i = (txn->offset >> SECTOR_SHIFT);
             (i < SECTORS_PER_FLASH_PG) &&
             (i < (txn->offset + txn->length) >> SECTOR_SHIFT);
             i++) {
            if (entry->bitmap & (1UL << i)) {
                memcpy(txn->data + (i << SECTOR_SHIFT),
                       entry->data + (i << SECTOR_SHIFT), SECTOR_SIZE);
            }
        }

        mutex_unlock(&entry->mutex);
        add_lru(cache, entry);
    }

    /* At this point, all cache entries for the request are locked and detached
     * from LRU so that they will not be evicted until we finish. Data buffers
     * for missed pages are also allocated. */
    iov_iter_init(&iter, get_local_var(req_iovecs), i, count);

    /* Write data into user buffers. */
    r = nvme_dma_write(req, &iter, count);

cleanup:
    list_for_each_entry_safe(txn, tmp, &hit_list, list)
    {
        struct cache_entry* entry = (struct cache_entry*)txn->opaque;
        struct data_cache* cache = get_cache_for_txn(txn);

        /* Cache hit. Release the lock now. */
        mutex_unlock(&entry->mutex);
        add_lru(cache, entry);

        list_del(&txn->list);
        SLABFREE(txn);
    }

    list_for_each_entry(txn, &req->txn_list, list)
    {
        /* Cache miss. Release the data buffer. */
        if (txn->data) free_mem(txn->data, FLASH_PG_BUFFER_SIZE);
    }

    return r;
}

int dc_process_request(struct user_request* req)
{
    int r;

    switch (req->req_type) {
    case IOREQ_WRITE:
        r = write_buffers(req);
        break;
    case IOREQ_READ:
        r = handle_cached_read(req);
        break;
    }

    return r;
}

void dc_init(size_t capacity)
{
    cache_init(&g_cache, 1, capacity / FLASH_PG_SIZE);
}

static void flush_cache(struct data_cache* cache)
{
    struct flash_transaction *txn, *tmp;
    struct cache_entry start_key, *entry;
    struct avl_iter iter;
    struct list_head wb_txns;

    INIT_LIST_HEAD(&wb_txns);

    start_key.lpa = 0;
    cache_avl_start_iter(cache, &iter, &start_key, AVL_GREATER_EQUAL);
    for (entry = cache_avl_get_iter(&iter); entry;) {
        if (entry->status == CES_DIRTY) {
            struct flash_transaction* wb_txn;
            int r;

            remove_lru(entry);
            mutex_lock(&entry->mutex);

            SLABALLOC(wb_txn);
            if (!wb_txn) continue;

            r = generate_writeback(wb_txn, cache, entry);
            if (r) {
                SLABFREE(wb_txn);
                continue;
            }

            list_add_tail(&wb_txn->list, &wb_txns);

            wb_txn->opaque = entry;
        }

        avl_inc_iter(&iter);
        entry = cache_avl_get_iter(&iter);
    }

    amu_dispatch(&wb_txns);

    list_for_each_entry_safe(txn, tmp, &wb_txns, list)
    {
        if (txn->opaque) {
            struct cache_entry* entry = (struct cache_entry*)txn->opaque;
            entry->status = CES_CLEAN;
            mutex_unlock(&entry->mutex);
            add_lru(cache, entry);
        }

        list_del(&txn->list);
        if (txn->data) free_mem(txn->data, FLASH_PG_BUFFER_SIZE);
        SLABFREE(txn);
    }
}

void dc_flush_ns(unsigned int nsid)
{
    struct data_cache* cache = get_cache_for_ns(nsid);
    flush_cache(cache);
}

void dc_flush_all(void) { flush_cache(&g_cache); }

void dc_report_stats(void)
{
    xil_printf("=============== Data cache ===============\n");
    xil_printf("  Total read hits: %d\n", g_cache.stats.total_read_hits);
    xil_printf("  Total read misses: %d\n", g_cache.stats.total_read_misses);
    xil_printf("==========================================\n");
}
