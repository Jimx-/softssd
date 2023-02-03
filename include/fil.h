#ifndef _FIL_H_
#define _FIL_H_

#include <list.h>
#include <spinlock.h>
#include <flash.h>

/* This must be compatible under both 32-bit and 64-bit. */
struct fil_task {
    struct flash_address addr; /* 24 */
    uint16_t type;             /* 26 */
    uint16_t source;           /* 28 */
    uint64_t lpa;              /* 36 */
    uint64_t data;             /* 44 */
    uint64_t code_buf;         /* 52 */
    uint32_t offset;           /* 56 */
    uint32_t length;           /* 60 */
    uint32_t code_length;      /* 64 */
    union {
        struct list_head queue;
        uint64_t __pad0[2];
    }; /* 80 */
    union {
        struct list_head waiting_list;
        uint64_t __pad1[2];
    };                  /* 96 */
    uint32_t completed; /* 100 */
#define FTS_OK    0
#define FTS_ERROR 1
    uint32_t status;        /* 104 */
    uint64_t opaque;        /* 112 */
    uint32_t total_xfer_us; /* 116 */
    uint32_t total_exec_us; /* 120 */
    uint64_t err_bitmap;    /* 128 */
} __attribute__((packed));

void fil_init(void);

#endif
