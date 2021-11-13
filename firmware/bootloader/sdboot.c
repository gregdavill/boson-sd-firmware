// This file is Copyright (c) 2014-2021 Florent Kermarrec <florent@enjoy-digital.fr>
// This file is Copyright (c) 2013-2014 Sebastien Bourdeauducq <sb@m-labs.hk>
// This file is Copyright (c) 2018 Ewen McNeill <ewen@naos.co.nz>
// This file is Copyright (c) 2018 Felix Held <felix-github@felixheld.de>
// This file is Copyright (c) 2019 Gabriel L. Somlo <gsomlo@gmail.com>
// This file is Copyright (c) 2017 Tim 'mithro' Ansell <mithro@mithis.com>
// This file is Copyright (c) 2018 William D. Jones <thor0505@comcast.net>
// This file is Copyright (c) 2021 Greg Davill <greg.davill@gmail.com>
// License: BSD

#include "sdboot.h"

#include <generated/csr.h>
#include <generated/mem.h>
#include <generated/soc.h>
#include <irq.h>
#include <libbase/crc.h>
#include <libbase/jsmn.h>
#include <libbase/uart.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <system.h>

#include "ff.h"
#include "flash.h"
#include "logger.h"
#include "sdcard.h"

/* Quick initial check to make sure binaries are actually for us */
#define MAGIC_MAIN 0xb5d930e9
#define MAGIC_BOOT 0x5d9ce906

/*-----------------------------------------------------------------------*/
/* Helpers                                                               */
/*-----------------------------------------------------------------------*/

#define max(x, y) (((x) > (y)) ? (x) : (y))
#define min(x, y) (((x) < (y)) ? (x) : (y))

void dump_bytes(unsigned int *ptr, int count, unsigned long addr);
void copy_file_from_ram_to_flash(uint32_t src_addr, uint32_t dst_addr, uint32_t dst_len);

static const char *put_rc(FRESULT rc) {
    const char *p;
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

#define FATFS_ERR(EXP)                                                        \
    do {                                                                      \
        FRESULT rc;                                                           \
        if ((rc = EXP) != FR_OK) {                                            \
            log_printf("FatFs: Error: \"" #EXP "\": FRESULT=%s", put_rc(rc)); \
        }                                                                     \
    } while (0);

#define NUMBER_OF_BYTES_ON_A_LINE 16
void dump_bytes(unsigned int *ptr, int count, unsigned long addr) {
    char *data = (char *)ptr;
    int line_bytes = 0, i = 0;

    fputs("Memory dump:", stdout);
    while (count > 0) {
        line_bytes =
            (count > NUMBER_OF_BYTES_ON_A_LINE) ? NUMBER_OF_BYTES_ON_A_LINE : count;

        printf("\n0x%08lx  ", addr);
        for (i = 0; i < line_bytes; i++)
            printf("%02x ", *(unsigned char *)(data + i));

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

/*-----------------------------------------------------------------------*/
/* Boot                                                                  */
/*-----------------------------------------------------------------------*/

void copy_file_from_ram_to_flash(uint32_t src_addr, uint32_t dst_addr, uint32_t dst_len) {
    log_printf("Boot: Copying 0x%08lx to flash_addr=0x%08lx (%ld bytes)", src_addr, dst_addr, dst_len);

    spiflash_set_high_perf_mode();

    uint8_t *data = (uint8_t *)src_addr;

    for (int addr = dst_addr; addr < dst_addr + dst_len; addr += 0x10000) {
        uint32_t len = addr + 0x10000 > (dst_addr + dst_len) ? (((dst_addr + dst_len) - addr) + 0xFF) & ~0xFF : 0x10000;
        uint32_t flash_address = addr;

        log_printf("Boot: Programming: addr=%08x, len=%08x", addr, len);

        /* First block in 64K erase block */
        spiflash_write_enable();
        spiflash_sector_erase(flash_address);

        /* While FLASH erase is in progress update LEDs */
        while (spiflash_read_status_register() & 1) {
        }

        for (int i = 0; i < len / 256; i++) {
            spiflash_write_enable();
            spiflash_page_program(flash_address, data, 256);
            flash_address += 256;
            data += 256;

            /* While FLASH erase is in progress update LEDs */
            while (spiflash_read_status_register() & 1) {
            }
        }
    }
    log_printf("Boot: FLASH loading complete");
}

/*-----------------------------------------------------------------------*/
/* SDCard Boot                                                           */
/*-----------------------------------------------------------------------*/

#if defined(CSR_SPISDCARD_BASE) || defined(CSR_SDCORE_BASE)

static uint32_t copy_file_from_sdcard_to_ram(const char *filename, unsigned long ram_address, uint32_t magic_check) {
    FRESULT fr;
    FATFS fs;
    FIL file;
    uint32_t br;
    uint32_t offset;
    unsigned long length;
    uint32_t crc_supplied;
    uint32_t crc_check;
    uint32_t magic_supplied;

    FATFS_ERR(fr = f_mount(&fs, "", 1));
    if (fr != FR_OK) {
        return 0;
    }
    FATFS_ERR(fr = f_open(&file, filename, FA_READ));
    if (fr != FR_OK) {
        log_printf("Boot: %s file not found", filename);
        FATFS_ERR(f_unmount(""));
        return 0;
    }

    length = f_size(&file);
    log_printf("Boot: Copy %s to 0x%08lx (%lu bytes)", filename, ram_address, length);
    offset = 0;
    for (;;) {
        FATFS_ERR(fr = f_read(&file, (void *)ram_address + offset, 0x8000, (UINT *)&br));
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

    memcpy((void *)&crc_supplied, (void *)ram_address + 4, 4);
    memcpy((void *)&magic_supplied, (void *)ram_address, 4);

    log_printf("Boot: Check %s: len=%lu bytes, crc=%08lx", filename, length, crc_supplied);

    /* magic check */
    if (magic_supplied == magic_check) {
        log_printf("Boot: Magic word passed, image is likely for us");
    } else {
        log_printf("Boot: Error: Magic word failed, image not likely for us. magic_file=0x%08lx, magic=0x%08lx", magic_supplied, magic_check);
        return 0;
    }

    /* CRC32 check */
    crc_check = crc32((void *)(ram_address + 8), length - 8);
    if (crc_check == crc_supplied) {
        log_printf("Boot: crc passed");
    } else {
        log_printf("Boot: crc fail, crc_supplied=0x%08lx, crc_check=0x%08lx", crc_supplied, crc_check);
        return 0;
    }

    return length;
}

static void sdcardboot_from_bin(const char *filename) {
    /* Copy Image from SDCard to address */
    uint32_t addr = 0x80000;
    log_printf("Boot: Loading %s @0x%08x", filename, addr);
    uint32_t length = copy_file_from_sdcard_to_ram(filename, HYPERRAM_BASE, MAGIC_MAIN);
    if (length == 0)
        return;

    /* Check if firmware matches FLASH? */
    if (memcmp((void *)HYPERRAM_BASE, (void *)SPIFLASH_BASE + addr, length) == 0) {
        log_printf("Boot: %s already matches FLASH", filename);
        return;
    }

    copy_file_from_ram_to_flash(HYPERRAM_BASE, addr, length);
    if (length == 0)
        return;
}

static void sdcardboot_from_json(const char *filename) {
    FRESULT fr;
    FATFS fs;
    FIL file;

    uint8_t i;
    uint8_t count;
    uint32_t length;

    /* FIXME: modify/increase if too limiting */
    char json_buffer[1024];
    char json_name[32];
    char json_value[32];

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

    FATFS_ERR(fr = f_read(&file, json_buffer, sizeof(json_buffer), (UINT *)&length));

    /* Close JSON file */
    FATFS_ERR(f_close(&file));
    FATFS_ERR(f_unmount("0:"));

    /* Parse JSON file */
    jsmntok_t t[32];
    jsmn_parser p;
    jsmn_init(&p);
    count = jsmn_parse(&p, json_buffer, strlen(json_buffer), t, sizeof(t) / sizeof(*t));
    for (i = 0; i < count - 1; i++) {
        memset(json_name, 0, sizeof(json_name));
        memset(json_value, 0, sizeof(json_value));
        /* Elements are JSON strings with 1 children */
        if ((t[i].type == JSMN_STRING) && (t[i].size == 1)) {
            /* Get Element's filename */
            memcpy(json_name, json_buffer + t[i].start, t[i].end - t[i].start);
            /* Get Element's address */
            memcpy(json_value, json_buffer + t[i + 1].start, t[i + 1].end - t[i + 1].start);

            /* Unlock bootloader sectors */
            if (strncmp(json_name, "bootloader_protection", 21) == 0) {
                if (strncmp(json_value, "disable", 7) == 0) {
                    spiflash_protection_clear();
					log_printf("Boot: Warning: Bootloader sectors unprotected");
                }
            } else {
                /* Copy Image from SDCard to address */
                uint32_t addr = strtoul(json_value, NULL, 0);
                log_printf("Boot: Loading %s @0x%08x", json_name, addr);
                uint32_t length = copy_file_from_sdcard_to_ram(json_name, HYPERRAM_BASE, (addr == 0) ? MAGIC_BOOT : MAGIC_MAIN);
                if (length == 0)
                    continue;

                /* Check if firmware matches FLASH? */
                if (memcmp((void *)HYPERRAM_BASE, (void *)SPIFLASH_BASE + addr, length) == 0) {
                    log_printf("Boot: %s already matches FLASH", json_name);
                    continue;
                }

                copy_file_from_ram_to_flash(HYPERRAM_BASE, addr, length);
            }
        }
    }
}

void sdcardboot(void) {
    log_printf("Boot: Using SDCard in SD-Mode");

    /* Boot from boson_sd_main.bin */
    log_printf("Boot: Checking boson_sd_main.bin");
    sdcardboot_from_bin("boson_sd_main.bin");

    /* Boot from boot.json */
    log_printf("Boot: Checking boot.json");
    sdcardboot_from_json("boot.json");

    log_printf("SDCard: Go IDLE");
    sdcard_go_idle();
    busy_wait(20);
}
#endif