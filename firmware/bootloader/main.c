#include <generated/csr.h>
#include <generated/mem.h>
#include <irq.h>
#include <stdint.h>
#include <stdio.h>
#include <uart.h>

#include "hyperram.h"
#include "logger.h"
#include "sdboot.h"
#include "timer.h"

/* prototypes */
void isr(void);

void isr(void) {
    __attribute__((unused)) unsigned int irqs;

    irqs = irq_pending() & irq_getmask();

    if (irqs & (1 << TIMER1_INTERRUPT))
        timer_isr();

#ifdef CSR_UART_BASE
#ifndef UART_POLLING
    if (irqs & (1 << UART_INTERRUPT))
        uart_isr();
#endif
#endif
}

/* Setting this to 'naked' removes 2 instructions which keep our stack clean for when we return. 
   Given we never return from this we can skip these. */
__attribute__((naked)) int main(int i, char **c) {
    /* Configure I/O for outputing UART */
    io_oe_out_write(0b01);

    /* Set LED to ON */
    leds_out_write(1);

    irq_setmask(0);
    irq_setie(1);
    uart_init();

    timer_init();

    printf("\n");
    log_printf("Info: Boson SD Bootloader");
    log_printf("Info: (c) Copyright 2021 Greg Davill");
    log_printf(
        "Info: built: "__DATE__
        " " __TIME__ "");

    if (sdphy_card_detect_read() == 1) {
        log_printf("SDCard: No card detected");
    } else {
        /* init HyperRAM */
        hyperram_init();

        busy_wait(500);

        /* Run bootloader logic */
        sdcardboot();
    }

    /* Set LED to OFF */
    leds_out_write(0);

    log_printf("Info: Bootloader Done, Restarting");
    uart_sync();

    busy_wait(10);

    /* Reboot into user gateware */
    while (1) {
        reset_out_write(1);
    }

    /* Let the compile know we'll never hit this point */
    __builtin_unreachable();

    return 0;
}
