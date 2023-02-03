#ifndef _ECC_PROTO_H_
#define _ECC_PROTO_H_

/* clock.c */
void perfcounter_init(int reset, int enable_divider);
static inline unsigned int get_cycle32(void)
{
    unsigned int value;
    asm volatile("MRC p15, 0, %0, c9, c13, 0\t\n" : "=r"(value));
    return value;
}

/* ipi.c */
int ipi_setup(void);
int ipi_trigger(void);

#endif
