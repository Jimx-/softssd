#include "proto.h"

static void report_stats(void) { ftl_report_stats(); }

static void handle_input(const u8* buf, size_t count)
{
    while (count > 0 && (buf[count - 1] == '\n' || buf[count - 1] == '\r'))
        count--;

    if (count == 6 && !memcmp(buf, "report", count)) {
        report_stats();
    }
}

void dbgcon_setup(void) { uart_set_recv_data_handler(handle_input); }
