#include <config.h>
#include <const.h>
#include <tls.h>
#include <proto.h>
#include <utils.h>

#define BOOT_TLS_OFFSET 0
size_t __tls_offset[NR_WORKER_THREADS] = {
    [0 ... NR_WORKER_THREADS - 1] = BOOT_TLS_OFFSET,
};

void tls_init(void)
{
    size_t size;
    char* ptr;
    int tid;
    extern char __tlsdata_start[], __tlsdata_end[];

    size = roundup(__tlsdata_end - __tlsdata_start, PG_SIZE);
    ptr = alloc_pages((size * NR_WORKER_THREADS) >> PG_SHIFT, ZONE_PS_DDR);

    for (tid = 0; tid < NR_WORKER_THREADS; tid++) {
        tls_offset(tid) = ptr - (char*)__tlsdata_start;
        memcpy(ptr, (void*)__tlsdata_start, __tlsdata_end - __tlsdata_start);

        ptr += size;
    }
}
