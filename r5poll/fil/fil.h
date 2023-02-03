#ifndef _FIL_FIL_H_
#define _FIL_FIL_H_

#include <list.h>
#include <flash.h>
#include <fil.h>

/* fil.c */
void fil_init(void);
int fil_is_channel_busy(unsigned int channel);
int fil_is_die_busy(unsigned int channel, unsigned int chip, unsigned int die,
                    int is_program);
void fil_dispatch(struct list_head* txn_list);
void fil_tick(void);

/* tsu.c */
void tsu_init(void);
void tsu_process_task(struct fil_task* task);
void tsu_notify_channel_idle(unsigned int channel);
void tsu_notify_chip_idle(unsigned int channel, unsigned int chip);
void tsu_flush_queues(void);

#endif
