#include <crc.h>
#include <generated/csr.h>
#include <generated/git.h>
#include <generated/mem.h>
#include <irq.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <uart.h>

#include "boson.h"
#include "ff.h"
#include "diskio.h"
#include "logger.h"
#include "timer.h"

/* prototypes */
void isr(void);
FRESULT scan_folders(char* path, UINT* cnt);
void boson_capture_configure(void);
void boson_capture_wait(void);

FRESULT scan_folders(
    char* path, /* Start node to be scanned (***also used as work area***) */
    UINT* cnt   /* Count of folders  */
) {
    FRESULT res;
    DIR dir;
    FILINFO fno;

    res = f_opendir(&dir, path); /* Open the directory */
    if (res == FR_OK) {
        for (;;) {
            res = f_readdir(&dir, &fno);                  /* Read a directory item */
            if (res != FR_OK || fno.fname[0] == 0) break; /* Break on error or end of dir */
            if (fno.fattrib & AM_DIR) {
                UINT _cnt = 0;
                if (strncmp("BSN", fno.fname, 3) == 0) {
                    int c = sscanf(fno.fname, "BSN%04u", &_cnt);
                    if (c && (_cnt > *cnt))
                        *cnt = _cnt;
                }
            }
        }
        f_closedir(&dir);
    }

    return res;
}

void dump_bytes(unsigned int* ptr, int count, unsigned long addr);

static const char* put_rc(FRESULT rc) {
    const char* p;
    static const char str[] =
        "OK\0DISK_ERR\0INT_ERR\0NOT_READY\0NO_FILE\0NO_PATH\0INVALID_NAME\0"
        "DENIED\0EXIST\0INVALID_OBJECT\0WRITE_PROTECTED\0INVALID_DRIVE\0"
        "NOT_ENABLED\0NO_FILE_SYSTEM\0MKFS_ABORTED\0TIMEOUT\0LOCKED\0"
        "NOT_ENOUGH_CORE\0TOO_MANY_OPEN_FILES\0FR_INVALID_PARAMETER\0";
    FRESULT i;

    for (p = str, i = 0; i != rc && *p; i++) {
        while (*p++)
            ;
    }

    return p;
}

#define FATFS_ERR(EXP)                                                                                                  \
    do {                                                                                                                \
        FRESULT rc;                                                                                                     \
        if ((rc = EXP) != FR_OK) {                                                                                      \
            log_printf("FatFs: Error: %s:%d:%s(): \"" #EXP "\": FRESULT=%s", __FILE__, __LINE__, __func__, put_rc(rc)); \
        }                                                                                                               \
    } while (0);

#define NUMBER_OF_BYTES_ON_A_LINE 16
void dump_bytes(unsigned int* ptr, int count, unsigned long addr) {
    char* data = (char*)ptr;
    int line_bytes = 0, i = 0;

    fputs("Memory dump:", stdout);
    while (count > 0) {
        line_bytes =
            (count > NUMBER_OF_BYTES_ON_A_LINE) ? NUMBER_OF_BYTES_ON_A_LINE : count;

        printf("\n0x%08lx  ", addr);
        for (i = 0; i < line_bytes; i++)
            printf("%02x ", *(unsigned char*)(data + i));

        for (; i < NUMBER_OF_BYTES_ON_A_LINE; i++)
            printf("   ");

        printf(" ");

        for (i = 0; i < line_bytes; i++) {
            if ((*(data + i) < 0x20) || (*(data + i) > 0x7e))
                printf(".");
            else
                printf("%c", *(data + i));
        }

        for (; i < NUMBER_OF_BYTES_ON_A_LINE; i++)
            printf(" ");

        data += (char)line_bytes;
        count -= line_bytes;
        addr += line_bytes;
    }
    printf("\n");
}

void boson_capture_configure(void) {
    uint32_t burst = (4e-6 * CONFIG_CLOCK_FREQUENCY);

    reader_reset_write(1);
    reader_source_mux_write(1);
    reader_external_sync_write(1);
    reader_burst_size_write(burst);
    reader_transfer_size_write(640 * 1024 / 4);
    reader_start_address_write(HYPERRAM_BASE >> 2);
    reader_enable_write(1);
}

void boson_capture_wait(void) {
    while (reader_done_read() == 0) {
    }
    reader_reset_write(1);
}

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

int main(int i, char** c) {
#ifdef CONFIG_CPU_HAS_INTERRUPT
    irq_setmask(0);
    irq_setie(1);

    timer_init();

#endif
#ifdef CSR_UART_BASE
    uart_init();
#endif

    busy_wait(25);

    log_printf("Boson SD Frame Grabber");
    log_printf("(c) Copyright 2021 Greg Davill");
    log_printf(
        " fw built: "__DATE__
        " " __TIME__ "");

    printf(" Migen git sha1: " MIGEN_GIT_SHA1);
    printf(" LiteX git sha1: " LITEX_GIT_SHA1);

    FATFS FatFs; /* FatFs work area needed for each volume */
    FIL Fil;     /* File object needed for each open file */
    UINT bw;
    FRESULT fr;

    uint8_t* ptr = (void*)HYPERRAM_BASE;

    log_printf("Boson: Init");
    boson_init();

    if (boson_resolution_hres_read() == 640 && boson_resolution_vres_read() == 512) {
        log_printf("Cap: Detected 640x512 Image stream");
    } else {
        log_printf("hres=%lu, vres=%lu", boson_resolution_hres_read(), boson_resolution_vres_read());
    }

    log_printf("Cap: boson_clk_freq=%lu", boson_boson_frequency_value_read());

    if (sdphy_card_detect_read() == 0) {
        log_printf("No SD Card inserted, please insert");
        while (sdphy_card_detect_read() == 0)
            ;
    }

    FATFS_ERR(fr = f_mount(&FatFs, "", 1)); /* Give a work area to the default drive */

    if (fr == FR_OK) {
        /* Find new dir and create */
        char path[255] = {0};
        UINT dir_cnt = 0;
        FATFS_ERR(fr = f_mkdir("DCIM"));

        FATFS_ERR(fr = scan_folders("DCIM/", &dir_cnt));
        dir_cnt += 1;
        log_printf("dir_cnt = %lu", dir_cnt);

        sprintf(path, "/DCIM/BSN%04u", dir_cnt);
        FATFS_ERR(fr = f_mkdir(path));

        for (unsigned int i = 0; i < 500; i++) {
            char name[64];
            sprintf(name, "%s/IMG_%04u.RAW", path, i);

            log_printf("%s", name);

            /* Capture from the Boson Stream */
            boson_capture_configure();
            boson_capture_wait();

            /* Flush caches because we've used the DMA, probably not needed, as the CPU never reads the Hyperram */
			// flush_cpu_dcache(); 
            // flush_l2_cache();

            /* Create our file */
            FATFS_ERR(fr = f_open(&Fil, name, FA_WRITE | FA_CREATE_ALWAYS));

            DWORD filesize = 640 * 1024;
            ptr = (void*)HYPERRAM_BASE;

            if (fr == FR_OK) {
				/* Handle all the FAT changes upfront */
                FATFS_ERR(fr = f_expand(&Fil, filesize, 1));

                if (0) {

                    /* Basic File writing, will typically write 64 sectors at a time, */
                    FATFS_ERR(fr = f_write(&Fil, ptr, filesize, &bw));
                    if (bw != filesize) {
                        log_printf("Cap: Error: file=%s, (bw=%lu)!=(filesize=%lu)\n", name, bw, filesize);
						continue;
                    }

                } else {

                    /* Accessing the contiguous file via low-level disk functions */
                    FIL* fp = &Fil;

                    /* Get physical location of the file data */
                    UINT drv = fp->obj.fs->pdrv;
                    LBA_t lba = fp->obj.fs->database + fp->obj.fs->csize * (fp->obj.sclust - 2);

                    /* Write sequential sectors from top of the file at a time */
					/* I don't think this function ever returns an error */
                    FATFS_ERR(disk_write(drv, ptr, lba, filesize / 512));

                }

                FATFS_ERR(fr = f_close(&Fil));
            }

            busy_wait(330);
        }
    }

    FATFS_ERR(f_unmount(""));

    while (1) {

    }

    return 0;
}
