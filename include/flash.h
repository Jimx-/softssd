#ifndef _FLASH_H_
#define _FLASH_H_

#include <stdint.h>
#include <string.h>

#include <types.h>
#include <list.h>

typedef uint64_t lpa_t;
/* 4G PPA * 16k page = 64 TB max. size */
typedef uint32_t ppa_t;
#define NO_LPA UINT64_MAX
#define NO_PPA UINT32_MAX

typedef uint64_t page_bitmap_t;

struct flash_address {
    unsigned int channel;
    unsigned int chip;
    unsigned int die;
    unsigned int plane;
    unsigned int block;
    unsigned int page;
};

/* Channel-Way-Die-Plane */
enum plane_allocate_scheme {
    PAS_CWDP,
    PAS_CWPD,
    PAS_CDWP,
    PAS_CDPW,
    PAS_CPWD,
    PAS_CPDW,
    PAS_WCDP,
    PAS_WCPD,
    PAS_WDCP,
    PAS_WDPC,
    PAS_WPCD,
    PAS_WPDC,
    PAS_DCWP,
    PAS_DCPW,
    PAS_DWCP,
    PAS_DWPC,
    PAS_DPCW,
    PAS_DPWC,
    PAS_PCWD,
    PAS_PCDW,
    PAS_PWCD,
    PAS_PWDC,
    PAS_PDCW,
    PAS_PDWC,
};

enum txn_type {
    TXN_READ,
    TXN_WRITE,
    TXN_ERASE,
};

enum txn_source {
    TS_USER_IO,
    TS_MAPPING,
    TS_GC,
};

struct flash_transaction {
    struct list_head list;
    struct list_head queue;
    struct list_head waiting_list;
    struct list_head page_movement_list;
    struct user_request* req;
    enum txn_type type;
    enum txn_source source;
    int worker;

    unsigned int nsid;
    lpa_t lpa;
    ppa_t ppa;
    struct flash_address addr;
    page_bitmap_t bitmap;
    int ppa_ready;
    void* opaque;

    uint8_t* data;
    unsigned int offset;
    unsigned int length;

    uint8_t* code_buf;
    unsigned int code_length;

    int completed;
    uint32_t total_xfer_us;
    uint32_t total_exec_us;
    uint64_t err_bitmap;
};

struct page_metadata {
    lpa_t lpa;
};

enum flash_command_code {
    FCMD_READ_PAGE,
    FCMD_READ_PAGE_MULTIPLANE,
    FCMD_PROGRAM_PAGE,
    FCMD_PROGRAM_PAGE_MULTIPLANE,
    FCMD_ERASE_BLOCK,
    FCMD_ERASE_BLOCK_MULTIPLANE,
};

#define MAX_CMD_ADDRS 4 /* For multiplane commands */
struct flash_command {
    enum flash_command_code cmd_code;
    unsigned int nr_addrs;
    struct flash_address addrs[MAX_CMD_ADDRS];
    struct page_metadata metadata[MAX_CMD_ADDRS];
};

static inline void flash_transaction_init(struct flash_transaction* txn)
{
    memset(txn, 0, sizeof(*txn));
}

#endif
