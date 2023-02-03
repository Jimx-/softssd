#ifndef _FIL_FLASH_CONFIG_H_
#define _FIL_FLASH_CONFIG_H_

#include <config.h>

#define FLASH_PG_SIZE     (16384)
#define FLASH_PG_OOB_SIZE (1872)
/* Full page size (user data + spare), rounded up to memory page size. */
#define FLASH_PG_BUFFER_SIZE \
    (roundup(FLASH_PG_SIZE + FLASH_PG_OOB_SIZE, PG_SIZE))

#define SECTORS_PER_FLASH_PG (FLASH_PG_SIZE >> SECTOR_SHIFT)

#define NR_CHANNELS       8
#define CHIPS_PER_CHANNEL 4
#define DIES_PER_CHIP     2
#define PLANES_PER_DIE    2
#define BLOCKS_PER_PLANE  1048
#define PAGES_PER_BLOCK   512

/* #define ENABLE_MULTIPLANE */

/* clang-format off */

#define CE_PINS \
    36, 20, 30, 27, \
    22, 28, 37, 43, \
    63, 58, 31, 21, \
    57, 65, 38, 54, \
    67, 64, 72, 61, \
    60, 66, 76, 73, \
    74, 71, 56, 59, \
    70, 69, 55, 75,

/* clang-format on */

#define WP_PINS 29, 26, 52, 53, 14, 24, 62, 68

#endif
