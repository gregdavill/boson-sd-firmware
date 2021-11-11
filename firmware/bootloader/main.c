#include <stdlib.h>
#include <stdint.h>
#include <generated/mem.h>
#include <generated/csr.h>

#include "time.h"

#include <irq.h>
#include <uart.h>

/* prototypes */
void isr(void);

uint32_t system_ticks;

void isr(void){
	__attribute__((unused)) unsigned int irqs;

	irqs = irq_pending() & irq_getmask();

  // if (irqs & (1 << TIMER0_INTERRUPT))
  // {
  //   system_ticks++;
  //   timer0_ev_pending_write(1);
  // }

#ifdef CSR_UART_BASE
#ifndef UART_POLLING
	if(irqs & (1 << UART_INTERRUPT))
		uart_isr();
#endif
#endif
}



/* Setting this to 'naked' removes 2 instructions which keep our stack clean for when we return. 
   Given we never return from this we can skip these. */
__attribute__((naked)) int main(int i, char **c)
{	

  io_oe_out_write(0b01);

  /* Set LED to ON */
	leds_out_write(1);

  irq_setmask(0);
	irq_setie(1);
	uart_init();

  time_init();
  
	printf("\n\n\n\e[92;1m    - Boson Booter - \e[0m\n");
 	printf("\n (c) Copyright 2021 Greg Davill \n");
 	printf(" bootloader built: "__DATE__ " " __TIME__ " \n\n");

  /* init HyperRAM */
  hyperram_init();

  printf("\n");

	sdcardboot();

  /* Set LED to OFF */
	leds_out_write(0);

  /* Reboot into user gateware */
  while(1){
    reset_out_write(1);
  }  

  /* Let the compile know we'll never hit this point */
  __builtin_unreachable();

	return 0;
}
