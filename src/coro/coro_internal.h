#ifndef _CORO_INTERNAL_H_
#define _CORO_INTERNAL_H_

#include "ucontext.h"

typedef enum {
    CR_DEAD,
    CR_BLOCKED,
    CR_RUNNABLE,
    CR_EXITING,
} coro_state_t;

struct __coro_tcb {
    coro_thread_t id;
    coro_state_t state;
    coro_attr_t attr;
    ucontext_t context;
    coro_mutex_t exitm;
    coro_cond_t exited;

    void* (*proc)(void*);
    void* arg;
    void* result;

    struct __coro_tcb* next;
};

typedef struct __coro_tcb coro_tcb_t;

#define MAIN_THREAD     ((coro_thread_t)-1)
#define NO_THREAD       ((coro_thread_t)-2)
#define NR_THREADS      4
#define MAX_THREADS     1024
#define is_valid_id(id) ((id == MAIN_THREAD) || (id >= 0 && id < nr_threads))

extern coro_thread_t current_thread;
extern coro_queue_t free_threads;
extern coro_queue_t run_queue;
extern coro_tcb_t main_thread;
extern coro_tcb_t** threads;
extern int nr_threads;

/* thread.c */
coro_tcb_t* coro_find_tcb(coro_thread_t thread);
void coro_thread_reset(coro_thread_t thread);

/* queue.c */
void coro_queue_init(coro_queue_t* queue);
int coro_queue_empty(coro_queue_t* queue);
void coro_queue_enqueue(coro_queue_t* queue, coro_thread_t thread);
coro_thread_t coro_queue_dequeue(coro_queue_t* queue);

/* scheduler.c */
void coro_schedule(void);
void coro_scheduler_init();
void coro_suspend(coro_state_t state);
void coro_unsuspend(coro_thread_t thread);

#endif
