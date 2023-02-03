#ifndef _SPINLOCK_H_
#define _SPINLOCK_H_

#include <stdint.h>

#include <utils.h>

typedef struct spinlock {
    volatile uint32_t lock;
} spinlock_t;

static inline void spinlock_init(spinlock_t* lock) { lock->lock = 0; }

static inline void spin_lock(spinlock_t* lock)
{
    while (__sync_lock_test_and_set(&lock->lock, 1)) {
#ifdef __aarch64__
        asm volatile("yield");
#endif
    }
#ifdef __aarch64__
    sevl();
    wfe();
#endif
}

static inline void spin_unlock(spinlock_t* lock)
{
    __sync_lock_release(&lock->lock);
}

#endif
