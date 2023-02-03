/* Address Mapping Unit */
#include "xil_assert.h"
#include "xdebug.h"
#include "ff.h"

#include <flash_config.h>
#include <list.h>
#include <flash.h>
#include <types.h>
#include <utils.h>
#include <proto.h>
#include <const.h>
#include <thread.h>
#include <avl.h>

#include <stdio.h>
#include <stdint.h>
#include <errno.h>

#define NAME "[AMU]"

#define GTD_FILENAME "gtd_ns%d.bin"

#define PAGES_PER_PLANE   (PAGES_PER_BLOCK * BLOCKS_PER_PLANE)
#define PAGES_PER_DIE     (PAGES_PER_PLANE * PLANES_PER_DIE)
#define PAGES_PER_CHIP    (PAGES_PER_DIE * DIES_PER_CHIP)
#define PAGES_PER_CHANNEL (PAGES_PER_CHIP * CHIPS_PER_CHANNEL)

/* Mapping virtual & physical page number. */
typedef uint32_t mvpn_t;
typedef uint32_t mppn_t;
#define NO_MPPN UINT32_MAX

#define XLATE_PG_SIZE                             \
    (FLASH_PG_SIZE * sizeof(struct xlate_entry) / \
     sizeof(struct xlate_entry_disk))

/* Translation entry */
/* Generally, flash page size (16k) can be larger than sector size (512B or 4k).
 * Thus, the host can write only a subset of sectors in a flash page. In that
 * case, we use the bitmap to record which sectors in a physical flash page are
 * valid. Upon writing a flash page, we check the sectors to be written with
 * sectors in the bitmap. If the set of sectors to be written covers all sectors
 * in the bitmap (which means all valid sectors stored in the old physical
 * page will be written), then we can skip reading the old PP. Otherwise reading
 * the old PP is required. */
struct xlate_entry {
    ppa_t ppa;
    uint32_t bitmap;
};

/* Strip page bitmap when an entry is saved on the disk to save space. Because
 * of this, the sizes of on-disk translation pages and in-memory translation
 * pages are different, which are represented as FLASH_PG_SIZE and
 * XLATE_PG_SIZE, respectively. */
struct xlate_entry_disk {
    ppa_t ppa;
};

/* Cached translation page. Contains (domain->xlate_ents_per_page) entries. */
struct xlate_page {
    mvpn_t mvpn;
    struct xlate_entry* entries;
    int dirty;
    mutex_t mutex;
    struct avl_node avl;
    struct list_head lru;
};

/* Translation page cache. Capacity = max # of xlate_page in the cache. */
struct xlate_pcache {
    size_t capacity;
    size_t size;
    struct avl_root root;
    struct list_head lru_list;
};

/* Per-namespace address mapping domain. */
struct am_domain {
    unsigned int nsid;
    size_t total_logical_pages;
    enum plane_allocate_scheme pa_scheme;

    /* Global translation directory */
    mppn_t* gtd;
    size_t xlate_ents_per_page;
    size_t total_xlate_pages;

    /* Translation page cache */
    struct xlate_pcache pcache;
};

static struct am_domain g_domain;

static void assign_plane(struct flash_transaction* txn);
static int alloc_page_for_mapping(struct flash_transaction* txn, mvpn_t mvpn,
                                  int for_gc);

/* Return the AM domain for a transaction based on its NSID. */
static inline struct am_domain* domain_get(struct flash_transaction* txn)
{
    /* TODO: more namespace */
    Xil_AssertNonvoid(txn->nsid == 1);
    return &g_domain;
}

/* Get the mapping virtual page number for an LPA. */
static inline mvpn_t get_mvpn(struct am_domain* domain, lpa_t lpa)
{
    return lpa / domain->xlate_ents_per_page;
}

/* Get the slot within a mapping virtual page for an LPA. */
static inline unsigned int get_mvpn_slot(struct am_domain* domain, lpa_t lpa)
{
    return lpa % domain->xlate_ents_per_page;
}

static inline void ppa_to_address(ppa_t ppa, struct flash_address* addr)
{
#define XLATE_PPA(ppa, name, cname)           \
    do {                                      \
        addr->name = ppa / PAGES_PER_##cname; \
        ppa = ppa % PAGES_PER_##cname;        \
    } while (0)
    XLATE_PPA(ppa, channel, CHANNEL);
    XLATE_PPA(ppa, chip, CHIP);
    XLATE_PPA(ppa, die, DIE);
    XLATE_PPA(ppa, plane, PLANE);
    XLATE_PPA(ppa, block, BLOCK);
    addr->page = ppa;
#undef XLATE_PPA
}

ppa_t address_to_ppa(struct flash_address* addr)
{
    return PAGES_PER_CHIP * (CHIPS_PER_CHANNEL * addr->channel + addr->chip) +
           PAGES_PER_DIE * addr->die + PAGES_PER_PLANE * addr->plane +
           PAGES_PER_BLOCK * addr->block + addr->page;
}

int amu_submit_transaction(struct flash_transaction* txn)
{
    int r;

    if (txn->type == TXN_READ && txn->data) {
        txn->code_buf = txn->data + FLASH_PG_SIZE;
        txn->code_length = FLASH_PG_OOB_SIZE;
    }

    r = submit_flash_transaction(txn);
    if (r) return r;

    if (txn->req) {
        switch (txn->type) {
        case TXN_READ:
            txn->req->stats.total_flash_read_txns++;
            txn->req->stats.total_flash_read_bytes +=
                txn->length + txn->code_length;
            txn->req->stats.flash_read_transfer_us += txn->total_xfer_us;
            txn->req->stats.flash_read_command_us += txn->total_exec_us;
            break;
        case TXN_WRITE:
            txn->req->stats.total_flash_write_txns++;
            txn->req->stats.total_flash_write_bytes += txn->length;
            txn->req->stats.flash_write_transfer_us += txn->total_xfer_us;
            txn->req->stats.flash_write_command_us += txn->total_exec_us;
            break;
        default:
            break;
        }
    }

    if (txn->type == TXN_READ && txn->err_bitmap) {
        if (txn->req) {
            txn->req->stats.ecc_error_blocks +=
                __builtin_popcountl(txn->err_bitmap);
        }

        /* Correction. */
        r = ecc_correct(txn->data, FLASH_PG_SIZE, txn->data + FLASH_PG_SIZE,
                        FLASH_PG_OOB_SIZE, txn->err_bitmap);
        if (r < 0) return -r;
    }

    return 0;
}

static int xpc_key_node_comp(void* key, struct avl_node* node)
{
    struct xlate_page* r1 = (struct xlate_page*)key;
    struct xlate_page* r2 = avl_entry(node, struct xlate_page, avl);

    if (r1->mvpn < r2->mvpn)
        return -1;
    else if (r1->mvpn > r2->mvpn)
        return 1;
    return 0;
}

static int xpc_node_node_comp(struct avl_node* node1, struct avl_node* node2)
{
    struct xlate_page* r1 = avl_entry(node1, struct xlate_page, avl);
    struct xlate_page* r2 = avl_entry(node2, struct xlate_page, avl);

    if (r1->mvpn < r2->mvpn)
        return -1;
    else if (r1->mvpn > r2->mvpn)
        return 1;
    return 0;
}

static inline void xpc_avl_start_iter(struct xlate_pcache* xpc,
                                      struct avl_iter* iter, void* key,
                                      int flags)
{
    avl_start_iter(&xpc->root, iter, key, flags);
}

static inline struct xlate_page* xpc_avl_get_iter(struct avl_iter* iter)
{
    struct avl_node* node = avl_get_iter(iter);
    if (!node) return NULL;
    return avl_entry(node, struct xlate_page, avl);
}

static struct xlate_page* xpc_find(struct xlate_pcache* xpc, mvpn_t mvpn)
{
    struct avl_node* node = xpc->root.node;
    struct xlate_page* xpg = NULL;

    while (node) {
        xpg = avl_entry(node, struct xlate_page, avl);

        if (xpg->mvpn == mvpn) {
            return xpg;
        } else if (mvpn < xpg->mvpn)
            node = node->left;
        else if (mvpn > xpg->mvpn)
            node = node->right;
    }

    return NULL;
}

static void xpc_init(struct xlate_pcache* xpc, size_t capacity)
{
    xpc->capacity = capacity;
    xpc->size = 0;
    INIT_LIST_HEAD(&xpc->lru_list);
    INIT_AVL_ROOT(&xpc->root, xpc_key_node_comp, xpc_node_node_comp);
}

static int xpc_add(struct xlate_pcache* xpc, mvpn_t mvpn,
                   struct xlate_page** xpgpp)
{
    struct xlate_page* xpg;

    if (xpc->size >= xpc->capacity) return ENOSPC;

    SLABALLOC(xpg);
    if (!xpc) return ENOMEM;

    memset(xpg, 0, sizeof(*xpg));
    xpg->mvpn = mvpn;
    xpg->dirty = FALSE;
    INIT_LIST_HEAD(&xpg->lru);
    mutex_init(&xpg->mutex, NULL);

    /* Allocate buffer for the translation page. Prefer PS DDR. */
    xpg->entries = alloc_pages(XLATE_PG_SIZE >> PG_SHIFT, ZONE_PS_DDR);
    if (!xpg->entries) {
        SLABFREE(xpg);
        return ENOMEM;
    }

    avl_insert(&xpg->avl, &xpc->root);
    xpc->size++;
    *xpgpp = xpg;

    return 0;
}

static inline void xpc_add_lru(struct xlate_pcache* xpc, struct xlate_page* xpg)
{
    list_add(&xpg->lru, &xpc->lru_list);
}

static inline void xpc_remove_lru(struct xlate_page* xpg)
{
    list_del(&xpg->lru);
}

static inline void xpc_update_mapping(struct xlate_page* xpg, unsigned int slot,
                                      ppa_t ppa, page_bitmap_t bitmap)
{
    xpg->entries[slot].ppa = ppa;
    xpg->entries[slot].bitmap |= bitmap;
    xpg->dirty = TRUE;
}

static int xpc_read_page(struct am_domain* domain, struct xlate_page* xpg)
{
    mvpn_t mvpn = xpg->mvpn;
    mppn_t mppn = domain->gtd[mvpn];
    struct flash_transaction txn;
    struct xlate_entry_disk* buf;
    int r;

    Xil_AssertNonvoid(!xpg->dirty);

    buf = alloc_pages(FLASH_PG_BUFFER_SIZE >> PG_SHIFT, ZONE_PS_DDR_LOW);
    if (!buf) return ENOMEM;

    flash_transaction_init(&txn);
    txn.type = TXN_READ;
    txn.source = TS_MAPPING;
    txn.nsid = domain->nsid;
    txn.lpa = mvpn;
    txn.ppa = mppn;
    txn.data = (u8*)buf;
    txn.offset = 0;
    txn.length = FLASH_PG_SIZE;
    txn.bitmap = (1UL << (FLASH_PG_SIZE >> SECTOR_SHIFT)) - 1;
    ppa_to_address(txn.ppa, &txn.addr);

    r = amu_submit_transaction(&txn);
    if (r == 0) {
        int i;
        for (i = 0; i < domain->xlate_ents_per_page; i++) {
            xpg->entries[i].ppa = buf[i].ppa;
            xpg->entries[i].bitmap = 0xffffffff;
        }
    }

    free_mem(buf, FLASH_PG_BUFFER_SIZE);
    return r;
}

static int xpc_flush_page(struct am_domain* domain, struct xlate_page* xpg)
{
    mvpn_t mvpn = xpg->mvpn;
    struct flash_transaction txn;
    struct xlate_entry_disk* buf;
    int i, r;

    if (!xpg->dirty) return 0;

    buf = alloc_pages(FLASH_PG_BUFFER_SIZE >> PG_SHIFT, ZONE_PS_DDR_LOW);
    if (!buf) return ENOMEM;

    for (i = 0; i < domain->xlate_ents_per_page; i++) {
        buf[i].ppa = xpg->entries[i].ppa;
    }

    flash_transaction_init(&txn);
    txn.type = TXN_WRITE;
    txn.source = TS_MAPPING;
    txn.nsid = domain->nsid;
    txn.lpa = mvpn;
    txn.ppa = NO_PPA;
    txn.data = (u8*)buf;
    txn.offset = 0;
    txn.length = FLASH_PG_SIZE;
    txn.bitmap = (1UL << (FLASH_PG_SIZE >> SECTOR_SHIFT)) - 1;

    assign_plane(&txn);
    alloc_page_for_mapping(&txn, mvpn, FALSE);

    r = amu_submit_transaction(&txn);
    if (r) goto out;

    xpg->dirty = FALSE;
    domain->gtd[mvpn] = txn.ppa;

    r = 0;

out:
    if (r) {
        bm_invalidate_page(&txn.addr);
    }

    free_mem(buf, FLASH_PG_BUFFER_SIZE);
    return r;
}

static int xpc_evict(struct xlate_pcache* xpc, struct xlate_page** xpgpp)
{
    struct xlate_page* xpg;

    if (list_empty(&xpc->lru_list)) return ENOMEM;

    xpg = list_entry(xpc->lru_list.prev, struct xlate_page, lru);

    list_del(&xpg->lru);
    avl_erase(&xpg->avl, &xpc->root);
    xpc->size--;

    *xpgpp = xpg;
    return 0;
}

/* Get the translation page referenced by mvpn from the page cache. Return
 * the page exclusively locked and detached from LRU. */
static int get_translation_page(struct am_domain* domain, mvpn_t mvpn,
                                struct xlate_page** xpgpp)
{
    struct xlate_page* xpg;
    struct xlate_pcache* xpc = &domain->pcache;
    int r;

    xpg = xpc_find(xpc, mvpn);

    if (xpg) {
        /* Cache hit. */
        xpc_remove_lru(xpg);
        mutex_lock(&xpg->mutex);
    } else {
        /* Try to add a new translation page. */
        r = xpc_add(xpc, mvpn, &xpg);

        if (unlikely(r == ENOSPC)) {
            /* Cache is full. */
            r = xpc_evict(xpc, &xpg);
            if (r) return r;

            if (xpg->dirty) {
                xpc_flush_page(domain, xpg);
            }

            xpg->mvpn = mvpn;
            avl_insert(&xpg->avl, &xpc->root);
            xpc->size++;

            mutex_lock(&xpg->mutex);
        } else if (unlikely(r != 0)) {
            /* Error. */
            return r;
        } else {
            /* Ok. */
            mutex_lock(&xpg->mutex);

            /* If there is no physical page for this translation page then
             * populate it with invalid PPAs. Otherwise issue a flash
             * transaction to read it from NAND flash. */
            if (domain->gtd[mvpn] == NO_MPPN) {
                int i;
                for (i = 0; i < domain->xlate_ents_per_page; i++) {
                    xpg->entries[i].ppa = NO_PPA;
                    xpg->entries[i].bitmap = 0;
                }
                xpg->dirty = TRUE;
            } else {
                r = xpc_read_page(domain, xpg);

                if (r) {
                    mutex_unlock(&xpg->mutex);
                    avl_erase(&xpg->avl, &xpc->root);
                    xpc_add_lru(&domain->pcache, xpg);
                }
            }
        }
    }

    *xpgpp = xpg;
    return 0;
}

/* Get the PPA associated with an LPA. Return NO_PPA if not mapped. */
static int get_ppa(struct am_domain* domain, lpa_t lpa, ppa_t* ppap,
                   page_bitmap_t* bitmapp)
{
    mvpn_t mvpn = get_mvpn(domain, lpa);
    unsigned int slot = get_mvpn_slot(domain, lpa);
    struct xlate_page* xpg;
    int r;

    r = get_translation_page(domain, mvpn, &xpg);
    if (r) return r;

    *ppap = xpg->entries[slot].ppa;
    if (bitmapp) *bitmapp = xpg->entries[slot].bitmap;

    mutex_unlock(&xpg->mutex);
    xpc_add_lru(&domain->pcache, xpg);

    return 0;
}

static void assign_plane(struct flash_transaction* txn)
{
    struct am_domain* domain = domain_get(txn);
    struct flash_address* addr = &txn->addr;
    lpa_t lpa = txn->lpa;

    static const unsigned int channel_count = NR_CHANNELS;
    static const unsigned int chip_count = CHIPS_PER_CHANNEL;
    static const unsigned int die_count = DIES_PER_CHIP;
    static const unsigned int plane_count = PLANES_PER_DIE;

#define ASSIGN_PHYS_ADDR(lpa, name)        \
    do {                                   \
        addr->name = (lpa) % name##_count; \
        (lpa) = (lpa) / name##_count;      \
    } while (0)

#define ASSIGN_PLANE(lpa, first, second, third, fourth) \
    do {                                                \
        ASSIGN_PHYS_ADDR(lpa, first);                   \
        ASSIGN_PHYS_ADDR(lpa, second);                  \
        ASSIGN_PHYS_ADDR(lpa, third);                   \
        ASSIGN_PHYS_ADDR(lpa, fourth);                  \
    } while (0)

    switch (domain->pa_scheme) {
    case PAS_CWDP:
        ASSIGN_PLANE(lpa, channel, chip, die, plane);
        break;
    case PAS_CWPD:
        ASSIGN_PLANE(lpa, channel, chip, plane, die);
        break;
    case PAS_CDWP:
        ASSIGN_PLANE(lpa, channel, die, chip, plane);
        break;
    case PAS_CDPW:
        ASSIGN_PLANE(lpa, channel, die, plane, chip);
        break;
    case PAS_CPWD:
        ASSIGN_PLANE(lpa, channel, plane, chip, die);
        break;
    case PAS_CPDW:
        ASSIGN_PLANE(lpa, channel, plane, die, chip);
        break;
    case PAS_WCDP:
        ASSIGN_PLANE(lpa, chip, channel, die, plane);
        break;
    case PAS_WCPD:
        ASSIGN_PLANE(lpa, chip, channel, plane, die);
        break;
    case PAS_WDCP:
        ASSIGN_PLANE(lpa, chip, die, channel, plane);
        break;
    case PAS_WDPC:
        ASSIGN_PLANE(lpa, chip, die, plane, channel);
        break;
    case PAS_WPCD:
        ASSIGN_PLANE(lpa, chip, plane, channel, die);
        break;
    case PAS_WPDC:
        ASSIGN_PLANE(lpa, chip, plane, die, channel);
        break;
    case PAS_DCWP:
        ASSIGN_PLANE(lpa, die, channel, chip, plane);
        break;
    case PAS_DCPW:
        ASSIGN_PLANE(lpa, die, channel, plane, chip);
        break;
    case PAS_DWCP:
        ASSIGN_PLANE(lpa, die, chip, channel, plane);
        break;
    case PAS_DWPC:
        ASSIGN_PLANE(lpa, die, chip, plane, channel);
        break;
    case PAS_DPCW:
        ASSIGN_PLANE(lpa, die, plane, channel, chip);
        break;
    case PAS_DPWC:
        ASSIGN_PLANE(lpa, die, plane, chip, channel);
        break;
    case PAS_PCWD:
        ASSIGN_PLANE(lpa, plane, channel, chip, die);
        break;
    case PAS_PCDW:
        ASSIGN_PLANE(lpa, plane, channel, die, chip);
        break;
    case PAS_PWCD:
        ASSIGN_PLANE(lpa, plane, chip, channel, die);
        break;
    case PAS_PWDC:
        ASSIGN_PLANE(lpa, plane, chip, die, channel);
        break;
    case PAS_PDCW:
        ASSIGN_PLANE(lpa, plane, die, channel, chip);
        break;
    case PAS_PDWC:
        ASSIGN_PLANE(lpa, plane, die, chip, channel);
        break;
    default:
        panic("Invalid plane allocation scheme");
        break;
    }
#undef ASSIGN_PLANE
#undef ASSIGN_PHYS_ADDR
}

static int update_read(struct flash_transaction* txn, struct xlate_entry* entry)
{
    struct flash_transaction read_txn;
    void* buf;
    page_bitmap_t bitmap = entry->bitmap & ~txn->bitmap;
    int r;

    Xil_AssertNonvoid(entry->ppa != NO_PPA);
    Xil_AssertNonvoid(bitmap);

    buf = alloc_pages(FLASH_PG_BUFFER_SIZE >> PG_SHIFT, ZONE_PS_DDR_LOW);
    if (!buf) return ENOMEM;

    flash_transaction_init(&read_txn);
    read_txn.type = TXN_READ;
    read_txn.req = txn->req;
    read_txn.source = TS_USER_IO;
    read_txn.nsid = txn->nsid;
    read_txn.lpa = txn->lpa;
    read_txn.ppa = entry->ppa;
    read_txn.data = (u8*)buf;
    read_txn.offset = (ffsl(bitmap) - 1) << SECTOR_SHIFT;
    read_txn.length = FLASH_PG_SIZE - read_txn.offset;
    read_txn.bitmap = bitmap;
    ppa_to_address(read_txn.ppa, &read_txn.addr);

    r = amu_submit_transaction(&read_txn);
    if (r == 0) {
        int i;
        for (i = 0; i < (FLASH_PG_SIZE >> SECTOR_SHIFT); i++) {
            if (bitmap & (1UL << i)) {
                memcpy(txn->data + (i << SECTOR_SHIFT),
                       buf + (i << SECTOR_SHIFT), SECTOR_SIZE);
            }
        }
    }

    free_mem(buf, FLASH_PG_BUFFER_SIZE);
    return r;
}

static int alloc_page_for_write(struct flash_transaction* txn, int for_gc)
{
    struct am_domain* domain = domain_get(txn);
    mvpn_t mvpn = get_mvpn(domain, txn->lpa);
    unsigned int slot = get_mvpn_slot(domain, txn->lpa);
    struct xlate_page* xpg;
    ppa_t ppa;
    int r;

    r = get_translation_page(domain, mvpn, &xpg);
    if (r) return r;

    ppa = xpg->entries[slot].ppa;

    if (ppa != NO_PPA) {
        /* Existing PP is found. We may need to read the old PP
         * (read-modify-write). */
        struct flash_address addr;
        page_bitmap_t bitmap = xpg->entries[slot].bitmap & txn->bitmap;

        if (bitmap != xpg->entries[slot].bitmap) {
            /* Update read required. */
            r = update_read(txn, &xpg->entries[slot]);
            if (r) goto out;
        }

        /* Invalidate the old PP. */
        ppa_to_address(ppa, &addr);
        bm_invalidate_page(&addr);
    }

    /* Allocate a new PP and update the mapping. */
    bm_alloc_page(txn->nsid, &txn->addr, for_gc, FALSE /* for_mapping */);
    txn->ppa = address_to_ppa(&txn->addr);
    xpc_update_mapping(xpg, slot, txn->ppa, txn->bitmap);

out:
    mutex_unlock(&xpg->mutex);
    xpc_add_lru(&domain->pcache, xpg);

    return r;
}

static int alloc_page_for_mapping(struct flash_transaction* txn, mvpn_t mvpn,
                                  int for_gc)
{
    struct am_domain* domain = domain_get(txn);
    mppn_t mppn = domain->gtd[mvpn];

    if (mppn != NO_MPPN) {
        struct flash_address addr;
        ppa_to_address((ppa_t)mppn, &addr);
        bm_invalidate_page(&addr);
    }

    bm_alloc_page(txn->nsid, &txn->addr, for_gc, TRUE /* for_mapping */);
    txn->ppa = address_to_ppa(&txn->addr);

    return 0;
}

/* Translate an LPA into PPA. Allocate a new physical page if not mapped. */
static int translate_lpa(struct flash_transaction* txn)
{
    struct am_domain* domain = domain_get(txn);
    int r;

    if (txn->type == TXN_READ) {
        ppa_t ppa;

        r = get_ppa(domain, txn->lpa, &ppa, NULL);
        if (r != 0) return r;

        if (ppa == NO_PPA) {
            /* Read an LP that has not been written. Just allocate a PP for
             * it without initialization. */
            assign_plane(txn);
            r = alloc_page_for_write(txn, FALSE);
            if (r != 0) return r;
        } else {
            txn->ppa = ppa;
            ppa_to_address(txn->ppa, &txn->addr);
        }
    } else {
        assign_plane(txn);
        r = alloc_page_for_write(txn, FALSE);
        if (r != 0) return r;
    }

    txn->ppa_ready = TRUE;
    return 0;
}

int amu_dispatch(struct list_head* txn_list)
{
    struct flash_transaction* txn;
    int r = 0;

    if (list_empty(txn_list)) return 0;

    list_for_each_entry(txn, txn_list, list)
    {
        r = translate_lpa(txn);
        if (r) goto cleanup;
    }

    /* All transactions are assigned PPAs -- it's time to execute them. */
    list_for_each_entry(txn, txn_list, list)
    {
        if (txn->ppa_ready) {
            amu_submit_transaction(txn);
        }
    }

cleanup:
    if (r) {
        list_for_each_entry(txn, txn_list, list)
        {
            if (txn->type == TXN_WRITE && txn->ppa_ready) {
                struct flash_address addr;
                ppa_to_address(txn->ppa, &addr);
                bm_invalidate_page(&addr);
            }
        }
    }

    return 0;
}

static int save_gtd(struct am_domain* domain, const char* filename)
{
    FIL fil;
    size_t write_size;
    UINT bw;
    int rc;

    write_size = domain->total_xlate_pages * sizeof(mppn_t);

    rc = f_open(&fil, filename, FA_CREATE_ALWAYS | FA_WRITE);
    if (rc) return EIO;

    rc = f_write(&fil, domain->gtd, write_size, &bw);
    if (rc || bw != write_size) return EIO;

    rc = f_close(&fil);
    return rc > 0 ? EIO : 0;
}

static int restore_gtd(struct am_domain* domain, const char* filename)
{
    FIL fil;
    size_t read_size;
    UINT br;
    int rc;

    read_size = domain->total_xlate_pages * sizeof(mppn_t);

    rc = f_open(&fil, filename, FA_READ);
    if (rc) return EIO;

    rc = f_read(&fil, domain->gtd, read_size, &br);
    if (rc || br != read_size) return EIO;

    rc = f_close(&fil);
    return rc > 0 ? EIO : 0;
}

static int domain_init(struct am_domain* domain, unsigned int nsid,
                       size_t capacity, size_t total_logical_pages,
                       enum plane_allocate_scheme pa_scheme, int reset)
{
    char gtd_filename[20];
    size_t xlate_ents_per_page =
        FLASH_PG_SIZE / sizeof(struct xlate_entry_disk);
    size_t total_xlate_pages =
        (total_logical_pages + xlate_ents_per_page - 1) / xlate_ents_per_page;
    size_t gtd_size;
    FILINFO fno;
    int i, r;

    domain->nsid = nsid;
    domain->total_logical_pages = total_logical_pages;
    domain->xlate_ents_per_page = xlate_ents_per_page;
    domain->total_xlate_pages = total_xlate_pages;
    domain->pa_scheme = pa_scheme;

    xpc_init(&domain->pcache, capacity);

    gtd_size = total_xlate_pages * sizeof(mppn_t);
    gtd_size = roundup(gtd_size, PG_SIZE);

    domain->gtd = alloc_pages(gtd_size >> PG_SHIFT, ZONE_PS_DDR);
    if (!domain->gtd) return ENOMEM;

    snprintf(gtd_filename, sizeof(gtd_filename), GTD_FILENAME, domain->nsid);

    if (f_stat(gtd_filename, &fno) != 0 || reset) {
        xil_printf(NAME " Initializing new global translation directory for "
                        "namespace %d ...",
                   nsid);

        for (i = 0; i < total_xlate_pages; i++) {
            domain->gtd[i] = NO_MPPN;
        }

        r = save_gtd(domain, gtd_filename);
        if (r) {
            panic(NAME " Failed to save GTD\n");
        } else {
            xil_printf("OK\n");
        }
    } else {
        xil_printf(NAME " Restoring global translation directory for namespace "
                        "%d (%d bytes) ...",
                   nsid, fno.fsize);
        r = restore_gtd(domain, gtd_filename);
        if (r) {
            panic(NAME " Failed to restore GTD\n");
        } else {
            xil_printf("OK\n");
        }
    }

    xil_printf(NAME " Initialized namespace %d with %lu logical pages\n", nsid,
               total_logical_pages);

    return 0;
}

void amu_init(size_t capacity, int reset)
{
    size_t total_logical_pages = (size_t)NR_CHANNELS * CHIPS_PER_CHANNEL *
                                 DIES_PER_CHIP * PLANES_PER_DIE *
                                 BLOCKS_PER_PLANE * PAGES_PER_BLOCK;

    domain_init(&g_domain, 1, capacity / XLATE_PG_SIZE, total_logical_pages,
                DEFAULT_PLANE_ALLOCATE_SCHEME, reset);
}

static void flush_domain(struct am_domain* domain)
{
    struct xlate_page start_key, *xpg;
    struct avl_iter iter;

    start_key.mvpn = 0;
    xpc_avl_start_iter(&domain->pcache, &iter, &start_key, AVL_GREATER_EQUAL);
    for (xpg = xpc_avl_get_iter(&iter); xpg;) {
        if (xpg->dirty) {
            xpc_remove_lru(xpg);
            mutex_lock(&xpg->mutex);

            xpc_flush_page(domain, xpg);

            mutex_unlock(&xpg->mutex);
            xpc_add_lru(&domain->pcache, xpg);
        }

        avl_inc_iter(&iter);
        xpg = xpc_avl_get_iter(&iter);
    }
}

static int save_domain(struct am_domain* domain)
{
    char gtd_filename[20];
    snprintf(gtd_filename, sizeof(gtd_filename), GTD_FILENAME, domain->nsid);

    flush_domain(domain);
    return save_gtd(domain, gtd_filename);
}

void amu_shutdown(void)
{
    int r;

    xil_printf(NAME " Saving AM domain for namespace %d ...", g_domain.nsid);
    r = save_domain(&g_domain);
    xil_printf("%s\n", r ? "FAILED" : "OK");
}
