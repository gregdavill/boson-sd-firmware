#include "timer.h"

#include <generated/csr.h>
#include <irq.h>

static uint32_t system_seconds;

void timer_isr(void) {
    system_seconds++;
    timer1_ev_pending_write(1);
}

void timer_init(void) {
    int t;

    timer1_en_write(0);
    timer1_ev_pending_write(1);
    timer1_ev_enable_write(1);
    irq_setmask(irq_getmask() | (1 << TIMER1_INTERRUPT));

    t = CONFIG_CLOCK_FREQUENCY;
    timer1_reload_write(t);
    timer1_load_write(t);
    timer1_en_write(1);
}

void timer_read(uint32_t* s, uint32_t* us) {
    /* Ensure that we didn't overflow the timer value while reading */
    do {
        *s = system_seconds;
        timer1_update_value_write(1);
        *us = (CONFIG_CLOCK_FREQUENCY - timer1_value_read()) / (int)(CONFIG_CLOCK_FREQUENCY / 1e6);
    } while (*s != system_seconds);
}