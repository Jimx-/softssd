#include <config.h>
#include <libcoro.h>
#include <thread.h>
#include <proto.h>
#include <const.h>
#include <flash.h>
#include <errno.h>

#define INIT_THREAD 0

#define WORKER_STACKSIZE 0x4000

#define NO_TIMEOUT UINT32_MAX

static struct worker_thread* self;
static struct worker_thread workers[NR_WORKER_THREADS];

static void (*thread_init_func)(void);
static int inited = FALSE;
static cond_t init_event;
static mutex_t init_event_mutex;

static int err_code;

static void* worker_main(void* arg);

struct worker_thread* worker_self(void) { return self; }

void worker_init(void (*init_func)(void))
{
    struct worker_thread* wp;
    coro_attr_t attr;

    thread_init_func = init_func;

    coro_attr_init(&attr);
    coro_attr_setstacksize(&attr, WORKER_STACKSIZE);

    if (mutex_init(&init_event_mutex, NULL) != 0) {
        panic("failed to initialize init event mutex");
    }
    if (cond_init(&init_event, NULL) != 0) {
        panic("failed to initialize init event");
    }

    for (wp = workers; wp < &workers[NR_WORKER_THREADS]; wp++) {
        wp->blocked_on = WT_BLOCKED_ON_NONE;
        wp->timeout_cycles = NO_TIMEOUT;

        if (mutex_init(&wp->event_mutex, NULL) != 0) {
            panic("failed to initialize mutex");
        }

        if (cond_init(&wp->event, NULL) != 0) {
            panic("failed to initialize condition variable");
        }

        if (coro_thread_create(&wp->tid, &attr, worker_main, wp) != 0) {
            panic("failed to start worker thread");
        }
    }

    self = NULL;
    worker_yield();
}

void worker_yield(void)
{
    coro_yield_all();
    self = NULL;
}

/* worker_sleep() as well as worker_wake() below must be called with interrupt
 * DISABLED. For worker_sleep(), it is only called by worker_wait_*() which
 * disables and enables interrupt before and after calling it. For
 * worker_wake(), it is mostly called by an interrupt handler on events to wake
 * up blocked worker. If worker_wake() is called outside an interrupt context,
 * care must be taken to disable interrupt before doing so. Similarly, you may
 * need to disable interrupt before calling worker_wait_*() to avoid
 * signal-before-wait situations. */
static void worker_sleep(void)
{
    struct worker_thread* thread = self;

    if (_mutex_lock(&thread->event_mutex) != 0) {
        panic("failed to lock event mutex");
    }

    if (_cond_wait(&thread->event, &thread->event_mutex) != 0) {
        panic("failed to wait on worker event");
    }

    if (mutex_unlock(&thread->event_mutex) != 0) {
        panic("failed to lock event mutex");
    }

    self = thread;
}

void worker_wake(struct worker_thread* worker, int why)
{
    if (why != WT_BLOCKED_ON_NONE && why != worker->blocked_on) return;

    if (_mutex_lock(&worker->event_mutex) != 0) {
        panic("failed to lock event mutex");
    }

    if (cond_signal(&worker->event) != 0) {
        panic("failed to signal worker event");
    }

    if (mutex_unlock(&worker->event_mutex) != 0) {
        panic("failed to lock event mutex");
    }
}

struct worker_thread* worker_suspend(int why)
{
    self->err_code = err_code;
    self->blocked_on = why;
    self->intr_flags = intr_save();

    return self;
}

void worker_resume(struct worker_thread* worker)
{
    self = worker;
    err_code = worker->err_code;
    self->blocked_on = WT_BLOCKED_ON_NONE;
    self->timeout_cycles = NO_TIMEOUT;

    intr_restore(self->intr_flags);
}

int worker_wait_timeout(int why, u32 timeout_ms)
{
    struct worker_thread* worker;

    worker = worker_suspend(why);

    /* Disable interrupt before setting up timeout so timer interrupt handler
     * will not see an inconsistent state. */
    intr_disable();

    if (timeout_ms == NO_TIMEOUT) {
        worker->timeout_cycles = NO_TIMEOUT;
    } else {
        worker->wait_start = timer_get_cycles();
        worker->timeout_cycles = timer_ms_to_cycles(timeout_ms);
    }

    worker->timed_out = FALSE;

    worker_sleep();

    /* Don't block interrupt for too long... */
    intr_enable();

    worker_resume(worker);

    return worker->timed_out ? ETIMEDOUT : 0;
}

void worker_wait(int why) { worker_wait_timeout(why, NO_TIMEOUT); }

void worker_check_timeout(void)
{
    struct worker_thread* wp;
    u32 cycles = timer_get_cycles();

    for (wp = workers; wp < workers + NR_WORKER_THREADS; wp++) {
        u32 delta;

        if (wp->blocked_on == WT_BLOCKED_ON_NONE) continue;

        if (wp->timeout_cycles == NO_TIMEOUT) continue;

        delta = cycles - wp->wait_start;
        wp->wait_start = cycles;

        if (wp->timeout_cycles <= delta) {
            /* Timed out. */
            xil_printf("Worker %d blocked on %d timed out\n", wp->tid,
                       wp->blocked_on);
            wp->timeout_cycles = NO_TIMEOUT;
            wp->timed_out = TRUE;
            worker_wake(wp, wp->blocked_on);
        } else {
            wp->timeout_cycles -= delta;
        }
    }
}

struct worker_thread* worker_get(thread_t tid)
{
    struct worker_thread* wp;

    for (wp = workers; wp < workers + NR_WORKER_THREADS; wp++) {
        if (wp->tid == tid) {
            return wp;
        }
    }

    return NULL;
}

static void* worker_main(void* arg)
{
    self = (struct worker_thread*)arg;

    if (self->tid == INIT_THREAD) {
        if (thread_init_func) {
            thread_init_func();
        }

        mutex_lock(&init_event_mutex);
        inited = TRUE;
        cond_broadcast(&init_event);
        mutex_unlock(&init_event_mutex);
    } else {
        mutex_lock(&init_event_mutex);
        while (!inited)
            cond_wait(&init_event, &init_event_mutex);
        mutex_unlock(&init_event_mutex);
    }

    /* xil_printf("Worker thread %p\n", arg); */

    nvme_worker_main();

    return NULL;
}
