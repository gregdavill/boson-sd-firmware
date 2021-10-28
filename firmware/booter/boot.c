// This file is Copyright (c) 2014-2021 Florent Kermarrec <florent@enjoy-digital.fr>
// This file is Copyright (c) 2013-2014 Sebastien Bourdeauducq <sb@m-labs.hk>
// This file is Copyright (c) 2018 Ewen McNeill <ewen@naos.co.nz>
// This file is Copyright (c) 2018 Felix Held <felix-github@felixheld.de>
// This file is Copyright (c) 2019 Gabriel L. Somlo <gsomlo@gmail.com>
// This file is Copyright (c) 2017 Tim 'mithro' Ansell <mithro@mithis.com>
// This file is Copyright (c) 2018 William D. Jones <thor0505@comcast.net>
// License: BSD

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <system.h>
#include <string.h>
#include <irq.h>

#include <generated/mem.h>
#include <generated/csr.h>
#include <generated/soc.h>

#include "sfl.h"
#include "boot.h"

#include <libbase/uart.h>

#include <libbase/console.h>
#include <libbase/crc.h>
#include <libbase/jsmn.h>
#include <libbase/progress.h>

#include <libliteeth/udp.h>
#include <libliteeth/tftp.h>

#include <liblitesdcard/spisdcard.h>
#include <liblitesdcard/sdcard.h>
#include <liblitesata/sata.h>
#include <libfatfs/ff.h>

/*-----------------------------------------------------------------------*/
/* Helpers                                                               */
/*-----------------------------------------------------------------------*/

#define max(x, y) (((x) > (y)) ? (x) : (y))
#define min(x, y) (((x) < (y)) ? (x) : (y))

uint32_t start_load_addr = 0;
uint32_t max_load_addr = 0;

/*-----------------------------------------------------------------------*/
/* Boot                                                                  */
/*-----------------------------------------------------------------------*/

extern void boot_helper(unsigned long r1, unsigned long r2, unsigned long r3, unsigned long addr);

void boot(unsigned long r1, unsigned long r2, unsigned long r3, unsigned long addr)
{
	
	max_load_addr -= HYPERRAM_BASE;
	uint8_t* data = (uint8_t*)start_load_addr;

	for(int addr = (start_load_addr - HYPERRAM_BASE); addr < max_load_addr; addr += 0x10000){

	uint32_t len = addr + 0x10000 > max_load_addr ? ((max_load_addr - addr) + 0xFF) & ~0xFF : 0x10000;
	uint32_t flash_address = addr;
	

		printf("%08x - %08x\n", addr, len);


	/* First block in 64K erase block */	
		spiflash_write_enable();
		spiflash_sector_erase(flash_address);

		/* While FLASH erase is in progress update LEDs */
		while(spiflash_read_status_register() & 1){ };

	
	for(int i = 0; i < len / 256; i++){

		spiflash_write_enable();
		spiflash_page_program(flash_address, data, 256);
		flash_address += 256;
		data += 256;


		/* While FLASH erase is in progress update LEDs */
		while(spiflash_read_status_register() & 1){ };
	}

	}

	printf("Executing booted program at 0x%08lx\n\n", SPIFLASH_BASE);
	printf("--============= \e[1mLiftoff!\e[0m ===============--\n");
	uart_sync();
	irq_setmask(0);
	irq_setie(0);
	flush_cpu_icache();
	flush_cpu_dcache();
	flush_l2_cache();


/* Perform a jump directly to our firmware located in the memory mapped SPIFLASH 
   * Note we configure SPI_BASE as the origin point of our firmware.
   * In reality it's actually located at an offset to skip over the bootloader and gateware.
   * 
   * Note 'SPIFLASH_BASE' is re-defined in boson-sd-bitstream before we compile this source.
   */
  void (*app)(void) = (void (*)(void))SPIFLASH_BASE;
  app();

}

enum {
	ACK_TIMEOUT,
	ACK_CANCELLED,
	ACK_OK
};

/*-----------------------------------------------------------------------*/
/* ROM Boot                                                              */
/*-----------------------------------------------------------------------*/

#ifdef ROM_BOOT_ADDRESS
/* Running the application code from ROM is the fastest way to execute code
   and could be interesting when the code is small enough, on large devices
   where many blockrams are available or simply when the execution speed is
   critical. Defining ROM_BOOT_ADDRESS in the SoC will make the BIOS jump to
   it at boot. */
void romboot(void)
{
	boot(0, 0, 0, ROM_BOOT_ADDRESS);
}
#endif

/*-----------------------------------------------------------------------*/
/* Serial Boot                                                           */
/*-----------------------------------------------------------------------*/

#ifdef CSR_UART_BASE

#define ACK_TIMEOUT_DELAY CONFIG_CLOCK_FREQUENCY/4
#define CMD_TIMEOUT_DELAY CONFIG_CLOCK_FREQUENCY/16

static void timer0_load(unsigned int value) {
	timer0_en_write(0);
	timer0_reload_write(0);
#ifndef CONFIG_DISABLE_DELAYS
	timer0_load_write(value);
#else
	timer0_load_write(0);
#endif
	timer0_en_write(1);
	timer0_update_value_write(1);
}

static int check_ack(void)
{
	int recognized;
	static const char str[SFL_MAGIC_LEN] = SFL_MAGIC_ACK;

	timer0_load(ACK_TIMEOUT_DELAY);
	recognized = 0;
	while(timer0_value_read()) {
		if(uart_read_nonblock()) {
			char c;
			c = uart_read();
			if((c == 'Q') || (c == '\e'))
				return ACK_CANCELLED;
			if(c == str[recognized]) {
				recognized++;
				if(recognized == SFL_MAGIC_LEN)
					return ACK_OK;
			} else {
				if(c == str[0])
					recognized = 1;
				else
					recognized = 0;
			}
		}
		timer0_update_value_write(1);
	}
	return ACK_TIMEOUT;
}

static uint32_t get_uint32(unsigned char* data)
{
	return ((uint32_t) data[0] << 24) |
			 ((uint32_t) data[1] << 16) |
			 ((uint32_t) data[2] << 8) |
			  (uint32_t) data[3];
}

#define MAX_FAILURES 256

/* Returns 1 if other boot methods should be tried */
int serialboot(void)
{
	struct sfl_frame frame;
	int failures;
	static const char str[SFL_MAGIC_LEN+1] = SFL_MAGIC_REQ;
	const char *c;
	int ack_status;

	printf("Booting from serial...\n");
	printf("Press Q or ESC to abort boot completely.\n");

	/* Send the serialboot "magic" request to Host and wait for ACK_OK */
	c = str;
	while(*c) {
		uart_write(*c);
		c++;
	}
	ack_status = check_ack();
	if(ack_status == ACK_TIMEOUT) {
		printf("Timeout\n");
		return 1;
	}
	if(ack_status == ACK_CANCELLED) {
		printf("Cancelled\n");
		return 0;
	}

	/* Assume ACK_OK */
	failures = 0;
	while(1) {
		int i;
		int timeout;
		int computed_crc;
		int received_crc;

		/* Get one Frame */
		i = 0;
		timeout = 1;
		while((i == 0) || timer0_value_read()) {
			if (uart_read_nonblock()) {
				if (i == 0) {
						timer0_load(CMD_TIMEOUT_DELAY);
						frame.payload_length = uart_read();
				}
				if (i == 1) frame.crc[0] = uart_read();
				if (i == 2) frame.crc[1] = uart_read();
				if (i == 3) frame.cmd    = uart_read();
				if (i >= 4) {
					frame.payload[i-4] = uart_read();
					if (i == (frame.payload_length + 4 - 1)) {
						timeout = 0;
						break;
					}
				}
				i++;
			}
			timer0_update_value_write(1);
		}

		/* Check Timeout */
		if (timeout) {
			/* Acknowledge the Timeout and continue with a new frame */
			uart_write(SFL_ACK_ERROR);
			continue;
		}

		/* Check Frame CRC */
		received_crc = ((int)frame.crc[0] << 8)|(int)frame.crc[1];
		computed_crc = crc16(&frame.cmd, frame.payload_length+1);
		if(computed_crc != received_crc) {
			/* Acknowledge the CRC error */
			uart_write(SFL_ACK_CRCERROR);

			/* Increment failures and exit when max is reached */
			failures++;
			if(failures == MAX_FAILURES) {
				printf("Too many consecutive errors, aborting");
				return 1;
			}
			continue;
		}

		/* Execute Frame CMD */
		switch(frame.cmd) {
			/* On SFL_CMD_ABORT ... */
			case SFL_CMD_ABORT:
				/* Reset failures */
				failures = 0;
				/* Acknowledge and exit */
				uart_write(SFL_ACK_SUCCESS);
				return 1;

			/* On SFL_CMD_LOAD... */
			case SFL_CMD_LOAD: {
				char *load_addr;

				/* Reset failures */
				failures = 0;

				/* Copy payload */
				load_addr = (char *)(uintptr_t) get_uint32(&frame.payload[0]);
				memcpy(load_addr, &frame.payload[4], frame.payload_length);

				if(start_load_addr == 0){
					start_load_addr = load_addr;
				}
				
				max_load_addr = load_addr + frame.payload_length;

				/* Acknowledge and continue */
				uart_write(SFL_ACK_SUCCESS);
				break;
			}
			/* On SFL_CMD_ABORT ... */
			case SFL_CMD_JUMP: {
				uint32_t jump_addr;

				/* Reset failures */
				failures = 0;

				/* Acknowledge and jump */
				uart_write(SFL_ACK_SUCCESS);
				jump_addr = get_uint32(&frame.payload[0]);

				printf("max_load_addr=%08x\n", max_load_addr);

				boot(0, 0, 0, jump_addr);
				break;
			}
			default:
				/* Increment failures */
				failures++;

				/* Acknowledge the UNKNOWN cmd */
				uart_write(SFL_ACK_UNKNOWN);

				/* Increment failures and exit when max is reached */
				if(failures == MAX_FAILURES) {
					printf("Too many consecutive errors, aborting");
					return 1;
				}

				break;
		}
	}
	return 1;
}

#endif
