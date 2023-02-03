#ifndef _CONFIG_H_
#define _CONFIG_H_

/* Uncomment this line to format EMMC on the first boot. */
/* #define FORMAT_EMMC */

/* Uncomment this line to wipe the entire SSD. */
/* #define WIPE_SSD */

/* Uncomment this line to perform a full bad block scan. */
/* #define FULL_BAD_BLOCK_SCAN */

/* Uncomment this line to wipe the mapping table. */
#define WIPE_MAPPING_TABLE

/* Uncomment this line to perform self-test on the NAND controller. */
/* #define NFC_SELFTEST */

/* Use histograms to track request statistics. */
#define USE_HISTOGRAM 1

#define PG_SHIFT 12
#define PG_SIZE  (1UL << PG_SHIFT)

/* System clock frequency. */
#define SYSTEM_HZ 25

#define CONFIG_NVME_IO_QUEUE_MAX 16

#define WORKERS_PER_QUEUE 8
#define NR_WORKER_THREADS (CONFIG_NVME_IO_QUEUE_MAX * WORKERS_PER_QUEUE)

#define CONFIG_STORAGE_CAPACITY_BYTES (512ULL << 30) /* 512 GiB */

#define SECTOR_SHIFT 12
#define SECTOR_SIZE  (1UL << SECTOR_SHIFT)

#define MAX_DATA_TRANSFER_SIZE 8 /* 256 pages */

#define CONFIG_DATA_CACHE_CAPACITY (512UL << 20) /* 512MB */

#define CONFIG_MAPPING_TABLE_CAPACITY (1UL << 30) /* 1GB */

#define DEFAULT_PLANE_ALLOCATE_SCHEME PAS_CWDP

#endif
