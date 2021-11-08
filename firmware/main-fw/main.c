#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <crc.h>
#include <stdint.h>
#include <stdbool.h>

#include "include/time.h"
#include "include/boson.h"

#include <generated/csr.h>
#include <generated/mem.h>
#include <generated/git.h>


#include "fatfs/source/ff.h"

#include <irq.h>
#include <uart.h>

uint32_t frame_count = 0;

/* prototypes */
void isr(void);
FRESULT scan_folders (char* path,UINT* cnt);
void boson_capture_configure(void);
void boson_capture_wait(void);

FRESULT scan_folders (
    char* path,        /* Start node to be scanned (***also used as work area***) */
	UINT* cnt		/* Count of folders  */
)
{
    FRESULT res;
    DIR dir;
    UINT i;
    FILINFO fno;


    res = f_opendir(&dir, path);                       /* Open the directory */
    if (res == FR_OK) {
        for (;;) {
            res = f_readdir(&dir, &fno);                   /* Read a directory item */
            if (res != FR_OK || fno.fname[0] == 0) break;  /* Break on error or end of dir */
            if (fno.fattrib & AM_DIR) {
//                printf("%s %s ", path, fno.fname);
				UINT _cnt = 0;
				if(strncmp("BSN", fno.fname, 3) == 0){
					int c = sscanf(fno.fname, "BSN%04u", &_cnt);
					if(_cnt > *cnt)
						*cnt = _cnt;
				}
            }
        }
        f_closedir(&dir);
    }

    return res;
}

#define FATFS_ERR(EXP) \
do {\
	if(EXP != FR_OK){ \
		printf( "fatfs: ("#EXP ")=%u\n", EXP); \
	} \
} while(0);


void boson_capture_configure(){
	uint32_t burst = (4e-6 * CONFIG_CLOCK_FREQUENCY);

	reader_reset_write(1);
	reader_source_mux_write(1);
	reader_external_sync_write(1);
	reader_burst_size_write(burst);
	reader_transfer_size_write(640*1024/4);
	reader_start_address_write(HYPERRAM_BASE>>2);
	reader_enable_write(1);
}

void boson_capture_wait(){
	while(reader_done_read() == 0);
	//reader_enable_write(0);
	reader_reset_write(1);
}


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


void isr(void){
	__attribute__((unused)) unsigned int irqs;

	irqs = irq_pending() & irq_getmask();

#ifdef CSR_UART_BASE
#ifndef UART_POLLING
	if(irqs & (1 << UART_INTERRUPT))
		uart_isr();

#endif
#endif
}


int main(int i, char **c)
{	

#ifdef CONFIG_CPU_HAS_INTERRUPT
	irq_setmask(0);
	irq_setie(1);
#endif
#ifdef CSR_UART_BASE
	uart_init();
#endif

	
	busy_wait(25);

	timer1_en_write(0);
	timer1_reload_write(-1);
	timer1_en_write(1);
	 
	printf("\e[92;1m    - Boson SD Frame Grabber - \e[0m\n");
 	printf("\n (c) Copyright 2021 Greg Davill \n");
 	printf(" fw built: "__DATE__ " " __TIME__ " \n\n");

 	printf("      Migen git sha1: "MIGEN_GIT_SHA1"\n");
 	printf("      LiteX git sha1: "LITEX_GIT_SHA1"\n");

	printf("--==========-- \e[1mBoson Init\e[0m ===========--\n");
 	boson_init();


	while(1){
		busy_wait(100);
		if(boson_resolution_hres_read() == 640 && boson_resolution_vres_read() == 512){
			printf("Detected 640x512 Image stream\n");	
			break;
		}
		else{
			printf("hres=%u, vres=%u\n", boson_resolution_hres_read(), boson_resolution_vres_read());
		}

		break;
	}


	//boson_capture_configure();


	for(int i = 0; i < 5; i++){
		printf("freq=%u\n", boson_boson_frequency_value_read());
		busy_wait(100);
	}

	

	FATFS FatFs;		/* FatFs work area needed for each volume */
	FIL Fil;			/* File object needed for each open file */
	UINT bw, br;
	FRESULT fr;

	volatile uint8_t* ptr = HYPERRAM_BASE;

	printf("&FatFs = %08x\n", &FatFs);
	printf("&Fil = %08x\n", &Fil);


	FATFS_ERR(fr = f_mount(&FatFs, "", 1));		/* Give a work area to the default drive */

	if (fr == FR_OK) {


		/* Find new dir and create */
		UINT dir_cnt = 0;
		fr = f_mkdir("DCIM");
		printf("f_mkdir() = %u\n",fr);

		FRESULT res;
		char path[255];

        res = scan_folders("DCIM/", &dir_cnt);
		dir_cnt += 1;
		printf("dir_cnt = %u\n", dir_cnt);

		sprintf(path, "/DCIM/BSN%04u", dir_cnt);
		fr = f_mkdir(path);
		printf("f_mkdir() = %u\n",fr);

		

		for( unsigned int i = 0; i < 3; i++){

			
			timer1_update_value_write(1);
			UINT t = timer1_value_read();

			char name[64];
			sprintf(name, "%s/IMG_%04u.RAW", path, i);

			printf("f_open() filename=%s -", name);

			while(1){
				boson_capture_configure();

				/* write speed */
				timer1_update_value_write(1);
				UINT t0 = timer1_value_read();

				boson_capture_wait();

				timer1_update_value_write(1);
				t0 = t0 - timer1_value_read();

				t0 /= (CONFIG_CLOCK_FREQUENCY / (int)1e6);
				printf("\n\n \e[93;1m[%01lu.%06lu]\e[0m \n", t0 / (int)1e6 , t0 % (int)1e6);
				
				/* Flush caches */
				flush_cpu_icache();
				flush_cpu_dcache();
				flush_l2_cache();

				busy_wait(100);
				
				dump_bytes(HYPERRAM_BASE, 256, HYPERRAM_BASE);
				//busy_wait(500);
				break;
			}
			

			fr = f_open(&Fil, name, FA_WRITE | FA_CREATE_ALWAYS);	/* Open a file */
			
			DWORD filesize = 640*1024;
			ptr = HYPERRAM_BASE;
			bool first = true;

			if (fr == FR_OK) {
				fr = f_expand(&Fil, filesize, 1);	
				while(filesize > 0){
					UINT xfer_size = 512;
					BYTE buff[512] __attribute__((aligned(4)));
					memcpy(buff, ptr, xfer_size);

					while(memcmp(buff, ptr, xfer_size) != 0){
						printf("memory error?! @%08x\n", ptr);
						memcpy(buff, ptr, xfer_size);
					}
					
					filesize -= xfer_size;
					ptr += xfer_size;

					fr = f_write(&Fil, buff, xfer_size, &br);
					
					//printf("f_write()=%u %u\n", fr, br);
					if ((fr != FR_OK) || (br != xfer_size)) {
						printf("f_write(): fr=%u,bw=%u\n", fr, br);
						break;
					}
				}

				FATFS_ERR(fr = f_sync(&Fil));
				//FATFS_ERR(fr = f_close(&Fil));
				
			}else{
				printf("f_open()=%u - ", fr);
			}

			timer1_update_value_write(1);
			t = t - timer1_value_read();

			t /= (CONFIG_CLOCK_FREQUENCY / (int)1e6);
			printf(" \e[92;1m[%01lu.%06lu]\e[0m\n", t / (int)1e6 , t % (int)1e6);
		}

	}

	

    while(1) {
			if(uart_read_nonblock()){
				char c = uart_read();
				if(c == 'b')
					reset_out_write(1);
			}

			//capture_service();
			//transmit_service();
			//hb_service();
		
	}
	
	return 0;
}



void hb_service()
{
	static int last_event;
	static int counter;

	if(elapsed(&last_event, CONFIG_CLOCK_FREQUENCY/100)) {
		leds_out_write(counter >= 5);
		if(++counter >= 10) {
			printf("frame_count: 0x%08x %u\n", frame_count, sdphy_card_detect_read());
			counter = 0;
		}
	}
}


enum {
	BUFFER_CLAIMED = 1,
	BUFFER_COMPLETE = 2,
};

uint32_t buffers[] = {
	0x00000000,
	0x02000000,
	0x04000000,
};

uint8_t buffer_owners[] = {0,0,0};


int32_t get_free_buffer(){
	for(int i = 0; i < 3; i++){
		if(!(buffer_owners[i] & BUFFER_CLAIMED)){
			if(!(buffer_owners[i] & BUFFER_COMPLETE)){
				return i;
			}
		}
	}

	for(int i = 0; i < 3; i++){
		if(!(buffer_owners[i] & BUFFER_CLAIMED)){
			return i;
		}
	}

	return -1;
}


int32_t get_complete_buffer(){
	for(int i = 0; i < 3; i++){
		if((buffer_owners[i] & BUFFER_COMPLETE)){
			return i;
		}
	}

	return -1;
}

