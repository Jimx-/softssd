#ifndef _R5_PROTO_H_
#define _R5_PROTO_H_

/* ipi.c */
int ipi_setup(void);
int ipi_trigger(void);

/* timer.c */
int timer_setup(void);
u64 timer_get_cycles(void);
u32 timer_ms_to_cycles(u32 ms);
u32 timer_us_to_cycles(u32 us);
u32 timer_cycles_to_us(u32 cycles);

#endif
