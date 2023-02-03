/**
 * hdr_malloc.h
 * Written by Filipe Oliveira and released to the public domain,
 * as explained at http://creativecommons.org/publicdomain/zero/1.0/
 *
 * Allocator selection.
 *
 * This file is used in order to change the HdrHistogram allocator at compile
 * time. Just define the following defines to what you want to use. Also add the
 * include of your alternate allocator if needed (not needed in order
 * to use the default libc allocator). */

#ifndef HDR_MALLOC_H__
#define HDR_MALLOC_H__

void* hdr_calloc(size_t num, size_t size);
void hdr_free(void* ptr);

#endif
