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

#include "boot.h"

#include <libbase/uart.h>

#include <libbase/console.h>
#include <libbase/crc.h>
#include <libbase/jsmn.h>
#include <libbase/progress.h>

#include <libliteeth/udp.h>
#include <libliteeth/tftp.h>

#include "ff.h"
#include "sdcard.h"

/*-----------------------------------------------------------------------*/
/* Helpers                                                               */
/*-----------------------------------------------------------------------*/

#define max(x, y) (((x) > (y)) ? (x) : (y))
#define min(x, y) (((x) < (y)) ? (x) : (y))

uint32_t start_load_addr = 0;
uint32_t max_load_addr = 0;



static char* put_rc (FRESULT rc)
{
	const char *p;
	static const char str[] =
		"OK\0DISK_ERR\0INT_ERR\0NOT_READY\0NO_FILE\0NO_PATH\0INVALID_NAME\0"
		"DENIED\0EXIST\0INVALID_OBJECT\0WRITE_PROTECTED\0INVALID_DRIVE\0"
		"NOT_ENABLED\0NO_FILE_SYSTEM\0MKFS_ABORTED\0TIMEOUT\0LOCKED\0"
		"NOT_ENOUGH_CORE\0TOO_MANY_OPEN_FILES\0FR_INVALID_PARAMETER\0";
	FRESULT i;

	for (p = str, i = 0; i != rc && *p; i++) {
		while(*p++);
	}

	return p;
}

#define FATFS_ERR(EXP) \
do {\
	FRESULT rc; \
	if((rc = EXP) != FR_OK){ \
		log_printf("FatFs: Error: \""#EXP "\": FRESULT=%s", put_rc(rc)); \
	} \
} while(0);


#define NUMBER_OF_BYTES_ON_A_LINE 16
void dump_bytes(unsigned int *ptr, int count, unsigned long addr)
{
	char *data = (char *)ptr;
	int line_bytes = 0, i = 0;

	fputs("Memory dump:", stdout);
	while (count > 0) {
		line_bytes =
			(count > NUMBER_OF_BYTES_ON_A_LINE)?
				NUMBER_OF_BYTES_ON_A_LINE : count;

		printf("\n0x%08lx  ", addr);
		for (i = 0; i < line_bytes; i++)
			printf("%02x ", *(unsigned char *)(data+i));

		for (; i < NUMBER_OF_BYTES_ON_A_LINE; i++)
			printf("   ");

		printf(" ");

		for (i = 0; i<line_bytes; i++) {
			if ((*(data+i) < 0x20) || (*(data+i) > 0x7e))
				printf(".");
			else
				printf("%c", *(data+i));
		}

		for (; i < NUMBER_OF_BYTES_ON_A_LINE; i++)
			printf(" ");

		data += (char)line_bytes;
		count -= line_bytes;
		addr += line_bytes;
	}
	printf("\n");
}

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
	

		log_printf("%08x - %08x", addr, len);


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

	log_printf("Executing booted program at 0x%08lx", SPIFLASH_BASE);
	log_printf("--============= \e[1mLiftoff!\e[0m ===============--");
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
/* SDCard Boot                                                           */
/*-----------------------------------------------------------------------*/

#if defined(CSR_SPISDCARD_BASE) || defined(CSR_SDCORE_BASE)

static int copy_file_from_sdcard_to_ram(const char * filename, unsigned long ram_address)
{
	FRESULT fr;
	FATFS fs;
	FIL file;
	uint32_t br;
	uint32_t offset;
	unsigned long length;

	FATFS_ERR(fr = f_mount(&fs, "", 1));
	if (fr != FR_OK){
		return 0;
	}
	FATFS_ERR(fr = f_open(&file, filename, FA_READ));
	if (fr != FR_OK) {
		log_printf("Boot: %s file not found", filename);
		FATFS_ERR(f_unmount(""));
		return 0;
	}

	FATFS_ERR(length = f_size(&file));
	log_printf("Copying %s to 0x%08lx (%ld bytes)", filename, ram_address, length);
	offset = 0;
	for (;;) {
		FATFS_ERR(fr = f_read(&file, (void*) ram_address + offset,  0x8000, (UINT *)&br));
		if (fr != FR_OK) {
			log_printf("Boot: File read error");
			FATFS_ERR(f_close(&file));
			FATFS_ERR(f_unmount(""));
			return 0;
		}
		if (br == 0)
			break;
		offset += br;
		
	}
	
	FATFS_ERR(f_close(&file));
	FATFS_ERR(f_unmount(""));

	return 1;
}

static void sdcardboot_from_json(const char * filename)
{
	FRESULT fr;
	FATFS fs;
	FIL file;

	uint8_t i;
	uint8_t count;
	uint32_t length;
	uint32_t result;

	/* FIXME: modify/increase if too limiting */
	char json_buffer[1024];
	char json_name[32];
	char json_value[32];

	unsigned long boot_r1 = 0;
	unsigned long boot_r2 = 0;
	unsigned long boot_r3 = 0;
	unsigned long boot_addr = 0;

	uint8_t image_found = 0;
	uint8_t boot_addr_found = 0;

	/* Read JSON file */
	FATFS_ERR(fr = f_mount(&fs, "", 1));
	if (fr != FR_OK)
		return;
	FATFS_ERR(fr = f_open(&file, filename, FA_READ));
	if (fr != FR_OK) {
		log_printf("Boot: %s file not found", filename);
		FATFS_ERR(f_unmount(""));
		return;
	}

	FATFS_ERR(fr = f_read(&file, json_buffer, sizeof(json_buffer), (UINT *) &length));

	/* Close JSON file */
	FATFS_ERR(f_close(&file));

	sdcard_go_inactive_state();
	FATFS_ERR(f_unmount("0:"));

	/* Parse JSON file */
	jsmntok_t t[32];
	jsmn_parser p;
	jsmn_init(&p);
	count = jsmn_parse(&p, json_buffer, strlen(json_buffer), t, sizeof(t)/sizeof(*t));
	for (i=0; i<count-1; i++) {
		memset(json_name,   0, sizeof(json_name));
		memset(json_value,  0, sizeof(json_value));
		/* Elements are JSON strings with 1 children */
		if ((t[i].type == JSMN_STRING) && (t[i].size == 1)) {
			/* Get Element's filename */
			memcpy(json_name, json_buffer + t[i].start, t[i].end - t[i].start);
			/* Get Element's address */
			memcpy(json_value, json_buffer + t[i+1].start, t[i+1].end - t[i+1].start);
			/* Skip bootargs (optional) */
			if (strncmp(json_name, "bootargs", 8) == 0) {
				continue;
			}
			/* Get boot addr (optional) */
			else if (strncmp(json_name, "addr", 4) == 0) {
				boot_addr = strtoul(json_value, NULL, 0);
				boot_addr_found = 1;
			}
			/* Get boot r1 (optional) */
			else if (strncmp(json_name, "r1", 2) == 0) {
				memcpy(json_name, json_buffer + t[i].start, t[i].end - t[i].start);
				boot_r1 = strtoul(json_value, NULL, 0);
			}
			/* Get boot r2 (optional) */
			else if (strncmp(json_name, "r2", 2) == 0) {
				boot_r2 = strtoul(json_value, NULL, 0);
			}
			/* Get boot r3 (optional) */
			else if (strncmp(json_name, "r3", 2) == 0) {
				boot_r3 = strtoul(json_value, NULL, 0);
			/* Copy Image from SDCard to address */
			} else {
				log_printf("Boot: Loading %s @0x%08x", json_name, strtoul(json_value, NULL, 0));
				result = copy_file_from_sdcard_to_ram(json_name, strtoul(json_value, NULL, 0));
				if (result == 0)
					return;
				image_found = 1;
				if (boot_addr_found == 0) /* Boot to last Image address if no bootargs.addr specified */
					boot_addr = strtoul(json_value, NULL, 0);
			}
		}
	}

	log_printf("SDCard: Go IDLE");
	sdcard_go_idle();
	busy_wait(20);

	/* Boot */
	if (image_found)
		boot(boot_r1, boot_r2, boot_r3, boot_addr);
}

static void sdcardboot_from_bin(const char * filename)
{
	uint32_t result;
	result = copy_file_from_sdcard_to_ram(filename, HYPERRAM_BASE);
	if (result == 0)
		return;
	//boot(0, 0, 0, MAIN_RAM_BASE);
}

void sdcardboot(void)
{
	log_printf("Boot: Using SDCard in SD-Mode");

	/* Boot from boot.json */
	log_printf("Boot: Checking boot.json");
	sdcardboot_from_json("boot.json");


	/* Boot failed if we are here... */
	log_printf("Boot: SDCard boot failed.");

}
#endif