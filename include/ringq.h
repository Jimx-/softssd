#ifndef _RINGQ_H_
#define _RINGQ_H_

#include <stdint.h>

struct ringq {
    uint32_t avail_head;
    uint32_t used_head;
    uint32_t avail_num;
    uint32_t used_num;
    uint32_t shadow_avail_tail;
    uint32_t shadow_used_tail;
    volatile uint32_t* avail_tail;
    volatile uint32_t* used_tail;
    volatile uint32_t* avail_ring;
    volatile uint32_t* used_ring;
};

static inline void ringq_init(struct ringq* ringq, u8* buf, size_t size)
{
    size_t qsize;

    ringq->avail_head = 0;
    ringq->used_head = 0;
    ringq->shadow_avail_tail = 0;
    ringq->shadow_used_tail = 0;
    ringq->avail_tail = (volatile uint32_t*)buf;
    ringq->used_tail = (volatile uint32_t*)&buf[4];

    qsize = (size - 8) >> 1;
    ringq->avail_ring = (volatile uint32_t*)&buf[8];
    ringq->used_ring = (volatile uint32_t*)&buf[8 + qsize];
    ringq->avail_num = qsize >> 2;
    ringq->used_num = qsize >> 2;
}

static inline void ringq_read_avail_tail(struct ringq* ringq)
{
    ringq->shadow_avail_tail =
        __atomic_load_n(ringq->avail_tail, __ATOMIC_ACQUIRE);
}

static inline void ringq_read_used_tail(struct ringq* ringq)
{
    ringq->shadow_used_tail =
        __atomic_load_n(ringq->used_tail, __ATOMIC_ACQUIRE);
}

static inline void ringq_write_avail_tail(struct ringq* ringq)
{
    __atomic_store_n(ringq->avail_tail, ringq->shadow_avail_tail,
                     __ATOMIC_RELEASE);
}

static inline void ringq_write_used_tail(struct ringq* ringq)
{
    __atomic_store_n(ringq->used_tail, ringq->shadow_used_tail,
                     __ATOMIC_RELEASE);
}

static inline uint32_t ringq_next_avail(struct ringq* ringq, uint32_t idx)
{
    idx++;
    if (idx >= ringq->avail_num) idx = 0;
    return idx;
}

static inline uint32_t ringq_next_used(struct ringq* ringq, uint32_t idx)
{
    idx++;
    if (idx >= ringq->used_num) idx = 0;
    return idx;
}

static void ringq_add_avail(struct ringq* ringq, uint32_t elem)
{
    ringq->avail_ring[ringq->shadow_avail_tail] = elem;
    ringq->shadow_avail_tail =
        ringq_next_avail(ringq, ringq->shadow_avail_tail);
}

static void ringq_add_used(struct ringq* ringq, uint32_t elem)
{
    ringq->used_ring[ringq->shadow_used_tail] = elem;
    ringq->shadow_used_tail = ringq_next_used(ringq, ringq->shadow_used_tail);
}

static int ringq_get_avail(struct ringq* ringq, uint32_t* elemp)
{
    uint32_t elem;
    if (ringq->avail_head == ringq->shadow_avail_tail) return 0;

    elem = ringq->avail_ring[ringq->avail_head];
    ringq->avail_head = ringq_next_avail(ringq, ringq->avail_head);
    *elemp = elem;

    return 1;
}

static int ringq_get_used(struct ringq* ringq, uint32_t* elemp)
{
    uint32_t elem;
    if (ringq->used_head == ringq->shadow_used_tail) return 0;

    elem = ringq->used_ring[ringq->used_head];
    ringq->used_head = ringq_next_used(ringq, ringq->used_head);
    *elemp = elem;

    return 1;
}

#endif
