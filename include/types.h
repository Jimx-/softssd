#ifndef _TYPES_H_
#define _TYPES_H_

#include <stdint.h>
#include <list.h>

typedef uint64_t lha_t; /* logical host address */
typedef uint64_t pda_t; /* physical device address */

struct user_request_stats {
    unsigned int total_flash_read_txns;
    unsigned int total_flash_write_txns;
    unsigned int total_flash_read_bytes;
    unsigned int total_flash_write_bytes;
    unsigned int ecc_error_blocks;
    unsigned int flash_read_transfer_us;
    unsigned int flash_write_transfer_us;
    unsigned int flash_read_command_us;
    unsigned int flash_write_command_us;
};

struct user_request {
    int req_type;
    unsigned int nsid;
    lha_t start_lba;
    unsigned int sector_count;
    unsigned int status;
    uint64_t prps[2];

    struct list_head txn_list;

    struct user_request_stats stats;
};

#endif
