#include <config.h>

#include "proto.h"
#include "const.h"
#include <utils.h>

#include <stddef.h>
#include <stdint.h>
#include <string.h>

static void* hdr_malloc(size_t size)
{
    void* ptr;

    size = roundup(size, 16);
    size += 16;
    ptr = slaballoc(size);

    if (!ptr) {
        size = roundup(size, PG_SIZE);
        ptr = alloc_pages(size >> PG_SHIFT, ZONE_PS_DDR);

        if (!ptr) return NULL;

        size |= 1;
    }

    *(uint32_t*)ptr = (uint32_t)size;
    return ptr + 16;
}

void* hdr_calloc(size_t num, size_t size)
{
    size_t alloc_size = num * size;
    void* ptr = hdr_malloc(alloc_size);
    if (!ptr) return NULL;

    memset(ptr, 0, alloc_size);
    return ptr;
}

void hdr_free(void* ptr)
{
    void* p = ptr - 16;
    uint32_t size = *(uint32_t*)p;

    if (size & 1)
        free_mem(p, size & ~1);
    else
        slabfree(p, size);
}
