#ifndef _HISTOGRAM_H_
#define _HISTOGRAM_H_

#include <config.h>

#include <stdint.h>

#if USE_HISTOGRAM

#include "hdrhistogram/hdr_histogram.h"

typedef struct hdr_histogram* histogram_t;

#define histogram_init   hdr_init
#define histogram_record hdr_record_value

static inline int histogram_print(histogram_t h,
                                  int32_t ticks_per_half_distance)
{
    return hdr_percentiles_print((struct hdr_histogram*)h,
                                 ticks_per_half_distance, 1.0, CLASSIC);
}

#else

typedef int histogram_t;

static inline int histogram_init(int64_t lowest_discernible_value,
                                 int64_t highest_trackable_value,
                                 int significant_figures, histogram_t* result)
{}

static inline int histogram_record(histogram_t h, int64_t value)
{
    return FALSE;
}

static inline int histogram_print(histogram_t h,
                                  int32_t ticks_per_half_distance)
{}

#endif

#endif
