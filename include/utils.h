#ifndef _UTILS_H_
#define _UTILS_H_

#include "xil_printf.h"

#define min(x, y) (((x) < (y)) ? (x) : (y))
#define max(x, y) (((x) > (y)) ? (x) : (y))

#define roundup(x, align) \
    (((x) % align == 0) ? (x) : (((x) + align) - ((x) % align)))
#define rounddown(x, align) ((x) - ((x) % align))

#define likely(x)   __builtin_expect(!!(x), 1)
#define unlikely(x) __builtin_expect(!!(x), 0)

#define sev()  __asm__("sev")
#define sevl() __asm__("sevl")
#define wfe()  __asm__("wfe")

#define __WARN(file, line, caller, format...)                          \
    do {                                                               \
        xil_printf("WARNING at %s:%d %p\n", (file), (line), (caller)); \
        xil_printf(format);                                            \
    } while (0)

#define WARN(condition, format...)                                           \
    ({                                                                       \
        int __ret_warn_on = !!(condition);                                   \
        if (unlikely(__ret_warn_on))                                         \
            __WARN(__FILE__, __LINE__, __builtin_return_address(0), format); \
        unlikely(__ret_warn_on);                                             \
    })

#define WARN_ONCE(condition, format...)                   \
    ({                                                    \
        static int __already_done;                        \
        int __ret_do_once = !!(condition);                \
                                                          \
        if (unlikely(__ret_do_once && !__already_done)) { \
            __already_done = 1;                           \
            WARN(1, format);                              \
        }                                                 \
        unlikely(__ret_do_once);                          \
    })

#endif
