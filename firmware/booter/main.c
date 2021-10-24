#include <stdlib.h>
#include <stdint.h>
#include <generated/mem.h>
#include <generated/csr.h>
#include "spi.h"



/* Setting this to 'naked' removes 2 instructions which keep our stack clean for when we return. 
   Given we never return from this we can skip these. */
__attribute__((naked)) int main(int i, char **c)
{	

  io_oe_out_write(0b01);

  /* Set LED to ON */
	leds_out_write(1);

  /* Switch to bitbang mode and poll SPI-ID then configure for Quad Mode */
  spiInit();
  spiSetQE();

  /* Switch back to Memory mapped operation, now operating with Quad-IO */
  spiFree();

  /* Set LED to OFF */
	leds_out_write(0);


  /* Perform a jump directly to our firmware located in the memory mapped SPIFLASH 
   * Note we configure SPI_BASE as the origin point of our firmware.
   * In reality it's actually located at an offset to skip over the bootloader and gateware.
   * 
   * Note 'SPIFLASH_BASE' is re-defined in boson-sd-bitstream before we compile this source.
   */
  void (*app)(void) = (void (*)(void))SPIFLASH_BASE;
  app();

  /* 
    The above code actually performs a Jump-And-Link, jal, Since it's assuming that you will 
    want to return from that function. If instead you wanted to perform a jr, you can with the following.
    
    Both produce the same code size, so I'm leaving the pure C code in place.
      asm volatile(
        "jr %0;" 
        :: "r"(SPIFLASH_BASE) : );
  */

  

  /* Let the compile know we'll never hit this point */
  __builtin_unreachable();

	return 0;
}
