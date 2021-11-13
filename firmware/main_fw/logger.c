
#include "logger.h"

#include <generated/csr.h>
#include <generated/soc.h>
#include <stdarg.h>
#include <stdio.h>

#include "timer.h"

void log_printf(const char *fmt, ...) {
    va_list va;

    /* Print Time */
    uint32_t s, us;
    timer_read(&s, &us);
    printf("\e[92;1m[%01lu.%06lu]\e[0m ", s, us);

    va_start(va, fmt);
    vprintf(fmt, va);
    va_end(va);

    printf("\n");
}