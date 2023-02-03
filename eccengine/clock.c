void perfcounter_init(int reset, int enable_divider)
{
    int value = 1;

    if (reset) value |= 6;
    if (enable_divider) value |= 8;

    value |= 16;

    asm volatile("MCR p15, 0, %0, c9, c14, 2\n\t" ::"r"(0x8000000f));
    asm volatile("MCR p15, 0, %0, c9, c12, 0\t\n" ::"r"(value));
    asm volatile("MCR p15, 0, %0, c9, c12, 1\t\n" ::"r"(0x8000000f));
    asm volatile("MCR p15, 0, %0, c9, c12, 3\t\n" ::"r"(0x8000000f));
}
