#ifndef _ECC_H_
#define _ECC_H_

/* This must be compatible under both 32-bit and 64-bit. */
struct ecc_task {
#define ECT_CALC    1
#define ECT_CORRECT 2
    uint32_t type;        /* 4 */
    uint64_t data;        /* 12 */
    uint32_t offset;      /* 16 */
    uint32_t length;      /* 20 */
    uint64_t code;        /* 28 */
    uint32_t code_length; /* 32 */
    uint32_t completed;   /* 36 */
#define ETS_OK        0
#define ETS_DEC_ERROR 1
#define ETS_NOSPC     2
    uint32_t status;     /* 40 */
    uint64_t opaque;     /* 48 */
    uint64_t err_bitmap; /* 56 */
    u8 __pad[8];         /* 64 */
} __attribute__((packed));

#endif
