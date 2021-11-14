#include <stdint.h>
#include <stdio.h>

#include <generated/csr.h>
#include <generated/mem.h>
#include <generated/soc.h>

#include <irq.h>
#include <uart.h>

#include "flash.h"
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
    log_printf(
        "Info: version: " CONFIG_REPO_GIT_DESC "");

    /* Ensure QSPI mode is on */
    spiflash_set_quad_enable();

    /* Check spiflash protection, and set if not protected */
    log_printf("Spiflash: Info: Bootloader sectors %s", spiflash_protection_read() ? "PROTECTED" : "UNPROTECTED");
    spiflash_protection_set();

    if (sdphy_card_detect_read() == 1) {
        log_printf("SDCard: No card detected");
    } else {
        busy_wait(300);
        
        /* init HyperRAM */
        int timeout = 10;
        while(--timeout)
        {
            if(hyperram_init() == 0)
                break;
            busy_wait(10);
        }
        if(timeout == 0){
            log_printf("Hyperram: Error: RAM Init failed, restarting");
            busy_wait(100);

            while (1) {
                reset_out_write(1);
            }
        }

        busy_wait(200);

        /* Run bootloader logic */
        sdcardboot();
    }

    /* Ensure that FLASH protection is enabled */
    spiflash_protection_set();
    
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
