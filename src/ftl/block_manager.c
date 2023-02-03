#include "xil_assert.h"
#include "ff.h"

#include <flash_config.h>
#include <flash.h>
#include <bitmap.h>
#include <list.h>
#include <utils.h>
#include <proto.h>
#include <const.h>

#include <errno.h>
#include <stddef.h>

#define NAME "[BM]"

/* Bitmap containing all free blocks in the planes. */
#define PLANE_INFO_FILE "planes.bin"
#define BAD_BLOCKS_FILE "badblks.bin"

#define BF_BAD     0x1
#define BF_MAPPING 0x2

struct block_data {
    struct list_head list;
    unsigned short block_id;
    unsigned int nr_invalid_pages;
    unsigned short page_write_index;
    unsigned int nsid;
    int flags;
    bitchunk_t invalid_page_bitmap[BITCHUNKS(PAGES_PER_BLOCK)];
};

struct plane_allocator {
    struct block_data* blocks;
    struct list_head free_list;
    unsigned int free_list_size;
    struct block_data* data_wf;
    struct block_data* gc_wf;
    struct block_data* mapping_wf;
};

static struct plane_allocator**** planes;

static inline struct plane_allocator* get_plane(struct flash_address* addr)
{
    Xil_AssertNonvoid(addr->channel < NR_CHANNELS);
    Xil_AssertNonvoid(addr->chip < CHIPS_PER_CHANNEL);
    Xil_AssertNonvoid(addr->die < DIES_PER_CHIP);
    Xil_AssertNonvoid(addr->plane < PLANES_PER_DIE);
    return &planes[addr->channel][addr->chip][addr->die][addr->plane];
}

static inline struct block_data* get_block_data(struct plane_allocator* plane,
                                                unsigned int block_id)
{
    return &plane->blocks[block_id];
}

static struct block_data* get_free_block(struct plane_allocator* plane,
                                         unsigned int nsid, int for_mapping)
{
    struct block_data* block;

    if (list_empty(&plane->free_list)) return NULL;

    block = list_entry(plane->free_list.next, struct block_data, list);
    list_del(&block->list);
    plane->free_list_size--;
    block->nsid = nsid;
    block->flags |= for_mapping ? BF_MAPPING : 0;

    return block;
}

static void init_plane(struct plane_allocator* plane)
{
    int i;

    INIT_LIST_HEAD(&plane->free_list);

    for (i = 0; i < BLOCKS_PER_PLANE; i++) {
        struct block_data* block = get_block_data(plane, i);

        block->nr_invalid_pages = 0;
        block->block_id = i;
        block->page_write_index = 0;
        block->flags = 0;
        INIT_LIST_HEAD(&block->list);
        memset(&block->invalid_page_bitmap, 0,
               sizeof(block->invalid_page_bitmap));
    }
}

static void init_plane_wf(struct plane_allocator* plane)
{
    plane->data_wf = get_free_block(plane, 1, FALSE);
    plane->gc_wf = get_free_block(plane, 1, FALSE);
    plane->mapping_wf = get_free_block(plane, 1, TRUE);
}

static void alloc_planes(void)
{
    size_t nr_ptrs, nr_planes, nr_blocks, alloc_size;
    void* buf;
    void** cur_ptr;
    struct plane_allocator* cur_plane;
    struct block_data* cur_block;
    int i, j, k, l;

    nr_planes =
        NR_CHANNELS * CHIPS_PER_CHANNEL * DIES_PER_CHIP * PLANES_PER_DIE;

    nr_blocks = nr_planes * BLOCKS_PER_PLANE;

    nr_ptrs =
        NR_CHANNELS + NR_CHANNELS * CHIPS_PER_CHANNEL * (1 + DIES_PER_CHIP);

    alloc_size = nr_ptrs * sizeof(void*) +
                 nr_planes * sizeof(struct plane_allocator) +
                 nr_blocks * sizeof(struct block_data);
    alloc_size = roundup(alloc_size, PG_SIZE);

    buf = alloc_pages(alloc_size >> PG_SHIFT, ZONE_PS_DDR);
    Xil_AssertVoid(buf != NULL);

    cur_ptr = (void**)buf;
    cur_plane = (struct plane_allocator*)(buf + nr_ptrs * sizeof(void*));
    cur_block =
        (struct block_data*)(buf + nr_ptrs * sizeof(void*) +
                             nr_planes * sizeof(struct plane_allocator));

    planes = (struct plane_allocator****)cur_ptr;
    cur_ptr += NR_CHANNELS;

    for (i = 0; i < NR_CHANNELS; i++) {
        planes[i] = (struct plane_allocator***)cur_ptr;
        cur_ptr += CHIPS_PER_CHANNEL;

        for (j = 0; j < CHIPS_PER_CHANNEL; j++) {
            planes[i][j] = (struct plane_allocator**)cur_ptr;
            cur_ptr += DIES_PER_CHIP;

            for (k = 0; k < DIES_PER_CHIP; k++) {
                planes[i][j][k] = cur_plane;
                cur_plane += PLANES_PER_DIE;

                for (l = 0; l < PLANES_PER_DIE; l++) {
                    struct plane_allocator* plane = &planes[i][j][k][l];
                    memset(plane, 0, sizeof(struct plane_allocator));

                    plane->blocks = cur_block;
                    cur_block += BLOCKS_PER_PLANE;

                    init_plane(&planes[i][j][k][l]);
                }
            }
        }
    }
}

static void reset_blocks(struct plane_allocator* plane)
{
    int i;

    for (i = 0; i < BLOCKS_PER_PLANE; i++) {
        struct block_data* block = get_block_data(plane, i);

        list_add(&block->list, &plane->free_list);
        plane->free_list_size++;
    }
}

static void reset_planes(void)
{
    int i, j, k, l;

    for (i = 0; i < NR_CHANNELS; i++) {
        for (j = 0; j < CHIPS_PER_CHANNEL; j++) {
            for (k = 0; k < DIES_PER_CHIP; k++) {
                for (l = 0; l < PLANES_PER_DIE; l++) {
                    reset_blocks(&planes[i][j][k][l]);
                }
            }
        }
    }
}

static int save_plane_info(void)
{
    FIL fil;
    u8 *buf, *wptr;
    int planes_per_page, count;
    UINT bw;
    int i, j, k, l;
    int rc, r = EIO;

    planes_per_page =
        PG_SIZE / (BITCHUNKS(BLOCKS_PER_PLANE) * sizeof(bitchunk_t));

    rc = f_open(&fil, PLANE_INFO_FILE, FA_CREATE_ALWAYS | FA_WRITE);
    if (!rc) rc = f_lseek(&fil, 0);
    if (rc) return EIO;

    buf = alloc_pages(1, ZONE_PS_DDR);

    wptr = buf;
    count = 0;

    for (i = 0; i < NR_CHANNELS; i++) {
        for (j = 0; j < CHIPS_PER_CHANNEL; j++) {
            for (k = 0; k < DIES_PER_CHIP; k++) {
                for (l = 0; l < PLANES_PER_DIE; l++) {
                    struct plane_allocator* plane = &planes[i][j][k][l];
                    bitchunk_t block_map[BITCHUNKS(BLOCKS_PER_PLANE)];
                    struct block_data* block;
                    int free_num = plane->free_list_size;

                    memset(block_map, 0, sizeof(block_map));

                    list_for_each_entry(block, &plane->free_list, list)
                    {
                        SET_BIT(block_map, block->block_id);
                        if (!--free_num) break;
                    }

                    /* Write frontiers assigned by not yet used. Add them
                     * to the free list too. */
                    if (plane->data_wf &&
                        plane->data_wf->page_write_index == 0) {
                        SET_BIT(block_map, plane->data_wf->block_id);
                    }
                    if (plane->mapping_wf &&
                        plane->mapping_wf->page_write_index == 0) {
                        SET_BIT(block_map, plane->mapping_wf->block_id);
                    }
                    if (plane->gc_wf && plane->gc_wf->page_write_index == 0) {
                        SET_BIT(block_map, plane->gc_wf->block_id);
                    }

                    if (count == planes_per_page) {
                        rc = f_write(&fil, buf, PG_SIZE, &bw);
                        if (rc || bw != PG_SIZE) goto out;

                        wptr = buf;
                        count = 0;
                    }

                    memcpy(wptr, block_map, sizeof(block_map));
                    wptr += sizeof(block_map);
                    count++;
                }
            }
        }
    }

    if (wptr != buf) {
        rc = f_write(&fil, buf, PG_SIZE, &bw);
        if (rc || bw != PG_SIZE) goto out;
    }

    rc = f_close(&fil);
    if (rc) goto out;

    r = 0;
out:
    free_mem(buf, PG_SIZE);

    return r;
}

static int restore_plane_info(void)
{
    FIL fil;
    u8 *buf, *rptr;
    int planes_per_page, count;
    UINT br;
    int i, j, k, l;
    int rc, r = EIO;

    planes_per_page =
        PG_SIZE / (BITCHUNKS(BLOCKS_PER_PLANE) * sizeof(bitchunk_t));

    rc = f_open(&fil, PLANE_INFO_FILE, FA_READ);
    if (!rc) rc = f_lseek(&fil, 0);
    if (rc) return EIO;

    buf = alloc_pages(1, ZONE_PS_DDR);

    rc = f_read(&fil, buf, PG_SIZE, &br);
    if (rc || br != PG_SIZE) goto out;

    rptr = buf;
    count = 0;

    for (i = 0; i < NR_CHANNELS; i++) {
        for (j = 0; j < CHIPS_PER_CHANNEL; j++) {
            for (k = 0; k < DIES_PER_CHIP; k++) {
                for (l = 0; l < PLANES_PER_DIE; l++) {
                    struct plane_allocator* plane = &planes[i][j][k][l];
                    bitchunk_t block_map[BITCHUNKS(BLOCKS_PER_PLANE)];
                    struct block_data* block;
                    int idx;

                    if (count == planes_per_page) {
                        rc = f_read(&fil, buf, PG_SIZE, &br);
                        if (rc || br != PG_SIZE) goto out;

                        rptr = buf;
                        count = 0;
                    }

                    memcpy(block_map, rptr, sizeof(block_map));

                    for (idx = 0; idx < BLOCKS_PER_PLANE; idx++) {
                        if (GET_BIT(block_map, idx)) {
                            block = get_block_data(plane, idx);
                            list_add_tail(&block->list, &plane->free_list);
                            plane->free_list_size++;
                        }
                    }

                    rptr += sizeof(block_map);
                    count++;
                }
            }
        }
    }

    rc = f_close(&fil);
    if (rc) goto out;

    r = 0;
out:
    free_mem(buf, PG_SIZE);

    return r;
}

static void assign_wf(void)
{
    int i, j, k, l;

    for (i = 0; i < NR_CHANNELS; i++) {
        for (j = 0; j < CHIPS_PER_CHANNEL; j++) {
            for (k = 0; k < DIES_PER_CHIP; k++) {
                for (l = 0; l < PLANES_PER_DIE; l++) {
                    init_plane_wf(&planes[i][j][k][l]);
                }
            }
        }
    }
}

void bm_alloc_page(unsigned int nsid, struct flash_address* addr, int for_gc,
                   int for_mapping)
{
    struct plane_allocator* plane = get_plane(addr);
    struct block_data* block;

    /* TODO: more namespaces */
    Xil_AssertVoid(nsid == 1);

    block = for_mapping ? plane->mapping_wf
                        : (for_gc ? plane->gc_wf : plane->data_wf);
    addr->block = block->block_id;
    addr->page = block->page_write_index++;

    if (block->page_write_index == PAGES_PER_BLOCK) {
        block = get_free_block(plane, nsid, for_mapping);

        if (for_mapping)
            plane->mapping_wf = block;
        else if (for_gc)
            plane->gc_wf = block;
        else
            plane->data_wf = block;

        /* TODO: check for GC */
    }
}

void bm_invalidate_page(struct flash_address* addr)
{
    struct plane_allocator* plane = get_plane(addr);
    struct block_data* block = get_block_data(plane, addr->block);

    block->nr_invalid_pages++;
    SET_BIT(block->invalid_page_bitmap, addr->page);
}

static void scan_bad_blocks(int full)
{
    int i, j, k, l, b, r;
    u8 *rbuf, *wbuf;
    struct flash_transaction txn;

    flash_transaction_init(&txn);
    rbuf = alloc_pages(FLASH_PG_BUFFER_SIZE >> PG_SHIFT, ZONE_PS_DDR_LOW);

    if (!full) {
        /* Read the bad block mark programmed by the manufacturer. */
        txn.addr.page = 0;
        txn.data = rbuf;
        txn.offset = FLASH_PG_SIZE;
        txn.length = 1;
        txn.type = TXN_READ;
        txn.source = TS_USER_IO;
    } else {
        wbuf = alloc_pages(FLASH_PG_BUFFER_SIZE >> PG_SHIFT, ZONE_PS_DDR_LOW);

        /* Prepare write buffer. */
        for (i = 0; i < FLASH_PG_SIZE; i++) {
            wbuf[i] = i & 0xff;
        }

        txn.addr.page = 0;
        txn.offset = 0;
        txn.length = FLASH_PG_SIZE;
        txn.source = TS_USER_IO;
    }

    for (i = 0; i < NR_CHANNELS; i++) {
        txn.addr.channel = i;
        for (j = 0; j < CHIPS_PER_CHANNEL; j++) {
            txn.addr.chip = j;
            for (k = 0; k < DIES_PER_CHIP; k++) {
                txn.addr.die = k;
                for (l = 0; l < PLANES_PER_DIE; l++) {
                    struct plane_allocator* plane = &planes[i][j][k][l];

                    txn.addr.plane = l;
                    for (b = 0; b < BLOCKS_PER_PLANE; b++) {
                        struct block_data* block = get_block_data(plane, b);
                        int bad = FALSE;

                        /* Already bad. */
                        if (block->flags & BF_BAD) continue;

                        txn.addr.block = b;

                        if (!full) {
                            r = submit_flash_transaction(&txn);
                            bad = (r != 0 || rbuf[0] == 0);
                        } else {
                            bad = TRUE;

                            do {
                                txn.type = TXN_WRITE;
                                txn.data = wbuf;
                                r = amu_submit_transaction(&txn);
                                if (r) break;

                                txn.type = TXN_READ;
                                txn.data = rbuf;
                                r = amu_submit_transaction(&txn);
                                if (r) break;

                                bad = FALSE;
                            } while (FALSE);
                        }

                        if (bad) {
                            /* Bad block. */
                            block->flags |= BF_BAD;
                            list_del(&block->list);
                        }
                    }
                }
            }
        }
    }

    free_mem(rbuf, FLASH_PG_BUFFER_SIZE);
    if (full) {
        free_mem(wbuf, FLASH_PG_BUFFER_SIZE);
    }
}

static int save_bad_blocks(void)
{
    FIL fil;
    u8 *buf, *wptr;
    int planes_per_page, count;
    UINT bw;
    int i, j, k, l, b;
    int rc, r = EIO;

    planes_per_page =
        PG_SIZE / (BITCHUNKS(BLOCKS_PER_PLANE) * sizeof(bitchunk_t));

    rc = f_open(&fil, BAD_BLOCKS_FILE, FA_CREATE_ALWAYS | FA_WRITE);
    if (!rc) rc = f_lseek(&fil, 0);
    if (rc) return EIO;

    buf = alloc_pages(1, ZONE_PS_DDR);

    wptr = buf;
    count = 0;

    for (i = 0; i < NR_CHANNELS; i++) {
        for (j = 0; j < CHIPS_PER_CHANNEL; j++) {
            for (k = 0; k < DIES_PER_CHIP; k++) {
                for (l = 0; l < PLANES_PER_DIE; l++) {
                    struct plane_allocator* plane = &planes[i][j][k][l];
                    bitchunk_t block_map[BITCHUNKS(BLOCKS_PER_PLANE)];

                    memset(block_map, 0, sizeof(block_map));

                    for (b = 0; b < BLOCKS_PER_PLANE; b++) {
                        struct block_data* block = get_block_data(plane, b);
                        if (block->flags & BF_BAD) SET_BIT(block_map, b);
                    }

                    if (count == planes_per_page) {
                        rc = f_write(&fil, buf, PG_SIZE, &bw);
                        if (rc || bw != PG_SIZE) goto out;

                        wptr = buf;
                        count = 0;
                    }

                    memcpy(wptr, block_map, sizeof(block_map));
                    wptr += sizeof(block_map);
                    count++;
                }
            }
        }
    }

    if (wptr != buf) {
        rc = f_write(&fil, buf, PG_SIZE, &bw);
        if (rc || bw != PG_SIZE) goto out;
    }

    rc = f_close(&fil);
    if (rc) goto out;

    r = 0;
out:
    free_mem(buf, PG_SIZE);

    return r;
}

static int restore_bad_blocks(void)
{
    FIL fil;
    u8 *buf, *rptr;
    int planes_per_page, count;
    UINT br;
    int i, j, k, l;
    int rc, r = EIO;

    planes_per_page =
        PG_SIZE / (BITCHUNKS(BLOCKS_PER_PLANE) * sizeof(bitchunk_t));

    rc = f_open(&fil, BAD_BLOCKS_FILE, FA_READ);
    if (!rc) rc = f_lseek(&fil, 0);
    if (rc) return EIO;

    buf = alloc_pages(1, ZONE_PS_DDR);

    rc = f_read(&fil, buf, PG_SIZE, &br);
    if (rc || br != PG_SIZE) goto out;

    rptr = buf;
    count = 0;

    for (i = 0; i < NR_CHANNELS; i++) {
        for (j = 0; j < CHIPS_PER_CHANNEL; j++) {
            for (k = 0; k < DIES_PER_CHIP; k++) {
                for (l = 0; l < PLANES_PER_DIE; l++) {
                    struct plane_allocator* plane = &planes[i][j][k][l];
                    bitchunk_t block_map[BITCHUNKS(BLOCKS_PER_PLANE)];
                    struct block_data* block;
                    int idx;

                    if (count == planes_per_page) {
                        rc = f_read(&fil, buf, PG_SIZE, &br);
                        if (rc || br != PG_SIZE) goto out;

                        rptr = buf;
                        count = 0;
                    }

                    memcpy(block_map, rptr, sizeof(block_map));

                    for (idx = 0; idx < BLOCKS_PER_PLANE; idx++) {
                        if (GET_BIT(block_map, idx)) {
                            block = get_block_data(plane, idx);
                            block->flags |= BF_BAD;
                            list_del(&block->list);
                        }
                    }

                    rptr += sizeof(block_map);
                    count++;
                }
            }
        }
    }

    rc = f_close(&fil);
    if (rc) goto out;

    r = 0;
out:
    free_mem(buf, PG_SIZE);

    return r;
}

static void wipe_blocks(void)
{
    int i, j, k, l, b;
    struct flash_transaction txn;

    flash_transaction_init(&txn);
    txn.addr.page = 0;
    txn.type = TXN_ERASE;
    txn.source = TS_USER_IO;

    for (i = 0; i < NR_CHANNELS; i++) {
        txn.addr.channel = i;
        for (j = 0; j < CHIPS_PER_CHANNEL; j++) {
            txn.addr.chip = j;
            for (k = 0; k < DIES_PER_CHIP; k++) {
                txn.addr.die = k;
                for (l = 0; l < PLANES_PER_DIE; l++) {
                    txn.addr.plane = l;
                    for (b = 0; b < BLOCKS_PER_PLANE; b++) {
                        txn.addr.block = b;
                        submit_flash_transaction(&txn);
                    }
                }
            }
        }
    }
}

void bm_init(int wipe, int full_scan)
{
    FILINFO fno;
    int r;

    alloc_planes();

    if (wipe) wipe_blocks();

    /* Recover free block information. */
    if (f_stat(PLANE_INFO_FILE, &fno) != 0 || wipe) {
        xil_printf(NAME " Reseting planes ...");
        reset_planes();
        save_plane_info();
        xil_printf("OK\n");
    } else {
        xil_printf(NAME " Restoring planes (%d bytes) ...", fno.fsize);
        r = restore_plane_info();
        if (r) {
            panic(NAME " Failed to restore plane info\n");
        } else {
            xil_printf("OK\n");
        }
    }

    if (f_stat(BAD_BLOCKS_FILE, &fno) != 0) {
        xil_printf(NAME " Scanning bad blocks ...");
        scan_bad_blocks(FALSE);
        save_bad_blocks();
        xil_printf("OK\n");
    } else {
        xil_printf(NAME " Restoring bad blocks (%d bytes) ...", fno.fsize);
        r = restore_bad_blocks();
        if (r) {
            panic(NAME " Failed to restore bad blocks\n");
        } else {
            xil_printf("OK\n");
        }
    }

    if (full_scan) {
        wipe_blocks();
        xil_printf(NAME " Scanning bad blocks (full) ...");
        scan_bad_blocks(TRUE);
        save_bad_blocks();
        wipe_blocks();
        xil_printf("OK\n");
    }

    /* Assign write frontier blocks. */
    assign_wf();
}

void bm_shutdown(void)
{
    xil_printf(NAME " Saving planes ...");
    save_plane_info();
    xil_printf("OK\n");
}
