#include <config.h>
#include "xil_mmu.h"

#include <errno.h>
#include <stddef.h>

#include <list.h>
#include <const.h>
#include "proto.h"

#define MINPAGES      1
#define MAXPAGES      12
#define MAX_FREEPAGES 128

#define freelist_index(nr_pages) ((nr_pages)-MINPAGES)

struct hole {
    struct hole* h_next;
    unsigned long h_base;
    size_t h_len;
};

struct freelist {
    struct list_head list;
    size_t count;
};

#define NR_HOLES 512

struct zonedat {
    unsigned long base;
    unsigned long limit;
    struct hole* hole_head;     /* pointer to first hole */
    struct hole* free_slots;    /* ptr to list of unused table slots */
    struct hole hole[NR_HOLES]; /* the hole table */

    struct freelist freelists[MAXPAGES - MINPAGES + 1];
};

static struct zonedat zones[MEMZONE_MAX];

static void free_mem_zone(struct zonedat* zone, void* base, unsigned long len);

static void delete_slot(struct zonedat* zone, struct hole* prev_ptr,
                        struct hole* hp);
static void merge_hole(struct zonedat* zone, struct hole* hp);

static inline void freelist_init(struct freelist* freelist)
{
    INIT_LIST_HEAD(&freelist->list);
    freelist->count = 0;
}

static inline void* freelist_alloc(struct freelist* freelist)
{
    struct list_head* next;

    if (freelist->count == 0) return NULL;
    Xil_AssertNonvoid(!list_empty(&freelist->list));

    next = freelist->list.next;
    list_del(next);
    freelist->count--;

    return (void*)next;
}

static inline struct freelist* freelist_get(struct zonedat* zone, void* base,
                                            unsigned long length)
{
    int nr_pages;
    struct freelist* freelist;

    if (((uintptr_t)base % PG_SIZE != 0) || (length % PG_SIZE != 0))
        return NULL;

    nr_pages = length >> PG_SHIFT;

    if (nr_pages < MINPAGES || nr_pages > MAXPAGES) return NULL;

    freelist = &zone->freelists[freelist_index(nr_pages)];
    if (freelist->count >= MAX_FREEPAGES) return NULL;

    return freelist;
}

static inline void freelist_free(struct freelist* freelist, void* base)
{
    struct list_head* head = (struct list_head*)base;
    list_add(head, &freelist->list);
    freelist->count++;
}

void mem_init(int mem_zone, unsigned long mem_start, size_t free_mem_size)
{
    struct hole* hp;
    struct zonedat* zone = &zones[mem_zone];
    int i;

    for (hp = &zone->hole[0]; hp < &zone->hole[NR_HOLES]; hp++) {
        hp->h_next = hp + 1;
        hp->h_base = 0;
        hp->h_len = 0;
    }

    zone->hole[NR_HOLES - 1].h_next = NULL;
    zone->hole_head = NULL;
    zone->free_slots = &zone->hole[0];
    zone->base = mem_start;
    zone->limit = mem_start + free_mem_size;

    for (i = MINPAGES; i <= MAXPAGES; i++) {
        freelist_init(&zone->freelists[freelist_index(i)]);
    }

    free_mem_zone(zone, (void*)mem_start, free_mem_size);
}

void* alloc_mem_zone(struct zonedat* zone, size_t memsize, size_t alignment)
{
    struct hole *hp, *prev_ptr;
    unsigned long old_base;

    prev_ptr = NULL;
    hp = zone->hole_head;
    while (hp != NULL) {
        size_t offset = 0;
        if (hp->h_base % alignment != 0)
            offset = alignment - (hp->h_base % alignment);
        if (hp->h_len >= memsize + offset) {
            old_base = hp->h_base + offset;
            hp->h_base += memsize + offset;
            hp->h_len -= (memsize + offset);
            if (prev_ptr && prev_ptr->h_base + prev_ptr->h_len == old_base)
                prev_ptr->h_len += offset;

            if (hp->h_len == 0) delete_slot(zone, prev_ptr, hp);

            return (void*)old_base;
        }

        prev_ptr = hp;
        hp = hp->h_next;
    }

    return NULL;
}

void* alloc_mem(size_t memsize, size_t alignment, int flags)
{
    int i;
    void* ptr;

    for (i = 0; i < MEMZONE_MAX; i++) {
        struct zonedat* zone;

        if (!(flags & (1 << i))) continue;
        zone = &zones[i];

        ptr = alloc_mem_zone(zone, memsize, alignment);
        if (ptr) return ptr;
    }

    return NULL;
}

void* alloc_pages(size_t nr_pages, int flags)
{
    int i;
    void* ptr;

    if (nr_pages >= MINPAGES && nr_pages <= MAXPAGES) {
        for (i = 0; i < MEMZONE_MAX; i++) {
            struct zonedat* zone;

            if (!(flags & (1 << i))) continue;
            zone = &zones[i];

            ptr = freelist_alloc(&zone->freelists[freelist_index(nr_pages)]);
            if (ptr) return ptr;
        }
    }

    return alloc_mem(nr_pages << PG_SHIFT, PG_SIZE, flags);
}

static void free_mem_zone(struct zonedat* zone, void* base, unsigned long len)
{
    struct hole *hp, *new_ptr, *prev_ptr;
    struct freelist* freelist;

    if (len == 0) return;

    freelist = freelist_get(zone, base, len);
    if (freelist) {
        freelist_free(freelist, base);
        return;
    }

    if ((new_ptr = zone->free_slots) == NULL) panic("hole table full");
    new_ptr->h_base = (unsigned long)base;
    new_ptr->h_len = len;
    zone->free_slots = new_ptr->h_next;
    hp = zone->hole_head;

    if (hp == NULL || (uintptr_t)base <= hp->h_base) {
        new_ptr->h_next = hp;
        zone->hole_head = new_ptr;
        merge_hole(zone, new_ptr);
        return;
    }

    prev_ptr = NULL;
    while (hp != NULL && (uintptr_t)base > hp->h_base) {
        prev_ptr = hp;
        hp = hp->h_next;
    }

    new_ptr->h_next = prev_ptr->h_next;
    prev_ptr->h_next = new_ptr;
    merge_hole(zone, prev_ptr);
}

void free_mem(void* base, unsigned long len)
{
    struct zonedat* zone;

    for (zone = zones; zone < &zones[MEMZONE_MAX]; zone++) {
        if (((uintptr_t)base >= zone->base) &&
            ((uintptr_t)base < zone->limit)) {
            free_mem_zone(zone, base, len);
            return;
        }
    }

    panic("freeing memory outside of any zone");
}

static void delete_slot(struct zonedat* zone, struct hole* prev_ptr,
                        struct hole* hp)
{
    if (hp == zone->hole_head)
        zone->hole_head = hp->h_next;
    else
        prev_ptr->h_next = hp->h_next;

    hp->h_next = zone->free_slots;
    hp->h_base = hp->h_len = 0;
    zone->free_slots = hp;
}

static void merge_hole(struct zonedat* zone, struct hole* hp)
{
    struct hole* next_ptr;

    if ((next_ptr = hp->h_next) == NULL) return;
    if (hp->h_base + hp->h_len == next_ptr->h_base) {
        hp->h_len += next_ptr->h_len;
        delete_slot(zone, hp, next_ptr);
    } else {
        hp = next_ptr;
    }

    if ((next_ptr = hp->h_next) == NULL) return;
    if (hp->h_base + hp->h_len == next_ptr->h_base) {
        hp->h_len += next_ptr->h_len;
        delete_slot(zone, hp, next_ptr);
    }
}
