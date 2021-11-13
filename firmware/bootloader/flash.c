/*
 *  Copyright 2020 Gregory Davill <greg.davill@gmail.com>
 *
 * Adapted from: https://github.com/norbertthiel
 * src: https://github.com/litex-hub/litespi/issues/52#issuecomment-890787356
 */

#include "flash.h"

#include <generated/csr.h>

#include "logger.h"

static uint8_t transfer_byte(uint8_t b) {
    // wait for tx ready
    while (!spiflash_core_master_status_tx_ready_read()) {
    };

    spiflash_core_master_rxtx_write((uint32_t)b);

    // wait for rx ready
    while (!spiflash_core_master_status_rx_ready_read()) {
    };

    return spiflash_core_master_rxtx_read();
}

static void transfer_cmd(uint8_t *bs, uint8_t *resp, int len) {
    spiflash_core_master_phyconfig_len_write(8);
    spiflash_core_master_phyconfig_width_write(1);
    spiflash_core_master_phyconfig_mask_write(1);
    spiflash_core_master_cs_write(1);

    if (resp != 0) {
        if (bs != 0) {
            for (int i = 0; i < len; i++) resp[i] = transfer_byte(bs[i]);
        } else {
            for (int i = 0; i < len; i++) resp[i] = transfer_byte(0xFF);
        }
    } else {
        for (int i = 0; i < len; i++) transfer_byte(bs[i]);
    }

    spiflash_core_master_cs_write(0);
}

static uint32_t spiflash_read_configuration_register(void) {
    uint8_t buf[3];
    transfer_cmd((uint8_t[]){0x15, 0, 0}, buf, 3);
    return buf[1] | (buf[2] << 8);
}

uint32_t spiflash_read_status_register(void) {
    busy_wait_us(500);
    uint8_t buf[2];
    transfer_cmd((uint8_t[]){0x05, 0}, buf, 2);
    return buf[1];
}

void spiflash_write_enable(void) {
    transfer_cmd((uint8_t[]){0x06}, 0, 1);
}

void spiflash_page_program(uint32_t addr, uint8_t *data, int len) {
    spiflash_core_master_phyconfig_len_write(8);
    spiflash_core_master_phyconfig_width_write(1);
    spiflash_core_master_phyconfig_mask_write(1);
    spiflash_core_master_cs_write(1);

    transfer_byte(0x02);
    transfer_byte(addr >> 16);
    transfer_byte(addr >> 8);
    transfer_byte(addr >> 0);

    spiflash_core_master_phyconfig_len_write(8);
    spiflash_core_master_phyconfig_width_write(1);
    spiflash_core_master_phyconfig_mask_write(1);

    for (int i = 0; i < len; i++) {
        while (!spiflash_core_master_status_tx_ready_read()) {
        }
        spiflash_core_master_rxtx_write((uint32_t)data[i]);

        while (!spiflash_core_master_status_rx_ready_read()) {
        }
        spiflash_core_master_rxtx_read();
    }

    spiflash_core_master_cs_write(0);
}

void spiflash_sector_erase(uint32_t addr) {
    spiflash_core_master_phyconfig_len_write(8);
    spiflash_core_master_phyconfig_width_write(1);
    spiflash_core_master_phyconfig_mask_write(1);
    spiflash_core_master_cs_write(1);

    transfer_byte(0xd8);
    transfer_byte(addr >> 16);
    transfer_byte(addr >> 8);
    transfer_byte(addr >> 0);

    spiflash_core_master_cs_write(0);
}

#define min(x, y) (((x) < (y)) ? (x) : (y))

int spiflash_write_stream(uint32_t addr, uint8_t *stream, int len) {
    int res = 0;
    if (((uint32_t)addr & ((64 * 1024) - 1)) == 0) {
        int w_len = min(len, SPIFLASH_MODULE_PAGE_SIZE);
        int offset = 0;
        while (w_len) {
            if ((((uint32_t)addr + offset) & ((64 * 1024) - 1)) == 0) {
                spiflash_write_enable();
                spiflash_sector_erase(addr + offset);

                while (spiflash_read_status_register() & 1) {
                }
            }

            spiflash_write_enable();
            spiflash_page_program(addr + offset, stream + offset, w_len);

            while (spiflash_read_status_register() & 1) {
            }

            offset += w_len;
            w_len = min(len - offset, SPIFLASH_MODULE_PAGE_SIZE);
            res = offset;
        }
    }
    return res;
}

/* 25Q128JV FLASH, Read Unique ID Number (4Bh)
        The Read Unique ID Number instruction accesses a factory-set read-only 64-bit number that is unique to
        each W25Q128JV device. The ID number can be used in conjunction with user software methods to help
        prevent copying or cloning of a system. The Read Unique ID instruction is initiated by driving the /CS pin
        low and shifting the instruction code “4Bh” followed by a four bytes of dummy clocks. After which, the 64-
        bit ID is shifted out on the falling edge of CLK
*/
void spiflash_read_uuid(uint8_t *uuid) {
    spiflash_core_master_phyconfig_len_write(8);
    spiflash_core_master_phyconfig_width_write(1);
    spiflash_core_master_phyconfig_mask_write(1);
    spiflash_core_master_cs_write(1);

    transfer_byte(0x4B);
    transfer_byte(0xFF);
    transfer_byte(0xFF);
    transfer_byte(0xFF);
    transfer_byte(0xFF);

    for (int i = 0; i < 8; i++) uuid[i] = transfer_byte(0xFF);

    spiflash_core_master_cs_write(0);
}

bool spiflash_protection_read(void) {
    uint8_t status = spiflash_read_status_register();
    if ((status & 0b00111100) != 0b00010000) {
        return false;
    }

    uint32_t config = spiflash_read_configuration_register();
    if ((config & 0x0008) != 0x0008) {
        log_printf("Spiflash: Warning: OTP bit \"Top/Bottom select\"=Top");
        return false;
    }

    return true;
}

void spiflash_protection_set(void) {

    uint8_t status = spiflash_read_status_register();
    uint32_t config = spiflash_read_configuration_register();
    if ((status & 0b00111100) == 0b00010000) {
        if ((config & 0x0008) == 0x0008) {
            return; // Already set
        }else{
           log_printf("Spiflash: Info: OTP bit \"Top/Bottom select\"=Top");
        }
    }

    log_printf("Spiflash: Info: Setting bootloader protection bits");

    status &= ~(0b00111100);
    status |= 0b00010000;

    spiflash_write_enable();

    spiflash_core_master_phyconfig_len_write(8);
    spiflash_core_master_phyconfig_width_write(1);
    spiflash_core_master_phyconfig_mask_write(1);
    spiflash_core_master_cs_write(1);

    transfer_byte(0x01);
    transfer_byte(status);

    if ((config & 0x0008) != 0x0008) {

        config |= 0x08;
        log_printf("Spiflash: Setting OTP bit \"Top/Bottom select\"=Bottom");
        transfer_byte(config & 0xFF);
        transfer_byte(config >> 8);
    }



    spiflash_core_master_cs_write(0);
    
    while (spiflash_read_status_register() & 1) {
        }

}


void spiflash_protection_clear(void) {

    uint8_t status = spiflash_read_status_register();
    if ((status & 0b00111100) == 0b00000000) {
        return;
    }


    log_printf("Spiflash: Info: Clearing bootloader protection bits");

    status &= ~(0b00111100);

    spiflash_write_enable();

    spiflash_core_master_phyconfig_len_write(8);
    spiflash_core_master_phyconfig_width_write(1);
    spiflash_core_master_phyconfig_mask_write(1);
    spiflash_core_master_cs_write(1);

    transfer_byte(0x01);
    transfer_byte(status);

    spiflash_core_master_cs_write(0);
    
    while (spiflash_read_status_register() & 1) {
        }

}

uint32_t spiflash_read_id(void) {
    uint32_t spi_id = 0;

    while (spiflash_read_status_register() & 1) {
        }

    spiflash_core_master_phyconfig_len_write(8);
    spiflash_core_master_phyconfig_width_write(1);
    spiflash_core_master_phyconfig_mask_write(1);
    spiflash_core_master_cs_write(1);

    transfer_byte(0x90);  // Read manufacturer ID
    transfer_byte(0x00);  // Dummy byte 1
    transfer_byte(0x00);  // Dummy byte 2
    transfer_byte(0x00);  // Dummy byte 3

    spi_id = (spi_id << 8) | transfer_byte(0x00);  // Manufacturer ID
    spi_id = (spi_id << 8) | transfer_byte(0x00);  // Device ID
    spiflash_core_master_cs_write(0);

    busy_wait(1);

    spiflash_core_master_cs_write(1);
    transfer_byte(0x9f);
    transfer_byte(0x00);  // Manufacturer ID (again)

    spi_id = (spi_id << 8) | transfer_byte(0x00);  // Memory Type
    spi_id = (spi_id << 8) | transfer_byte(0x00);  // Memory Size
    spiflash_core_master_cs_write(0);

    return spi_id;
}

void spiflash_set_high_perf_mode(void) {
    uint32_t config = spiflash_read_configuration_register();

    if((config & 0x0200) == 0){
        log_printf("Spiflash: Info: Setting high-performance bit");

        config |= 0x0200;
        uint8_t status = spiflash_read_status_register();

        // Enable Write-Enable Latch (WEL)
        spiflash_write_enable();

        spiflash_core_master_phyconfig_len_write(8);
        spiflash_core_master_phyconfig_width_write(1);
        spiflash_core_master_phyconfig_mask_write(1);
        spiflash_core_master_cs_write(1);

        transfer_byte(0x01);
        transfer_byte(status);
        transfer_byte(config & 0xFF);
        transfer_byte(status >> 8);

        spiflash_core_master_cs_write(0);

        while (spiflash_read_status_register() & 1) {
        }
    }
}

void spiflash_set_quad_enable(void) {
    uint32_t id = spiflash_read_id();

    /* MX25R1635F */
    if (id == 0xc2152815) {
        // Set QE bit if not set
        log_printf("Spiflash: Info: Checking spiflash QE bit");

        // READ status register
        uint8_t status1 = spiflash_read_status_register();

        // Check Quad Enable bit
        if ((status1 & 0x40) == 0) {
            log_printf("Spiflash: Info: Setting spiflash QE bit");

            // Enable Write-Enable Latch (WEL)
            spiflash_write_enable();

            // Write back status1 and status2 with QE bit set
            status1 |= 0x40;

            spiflash_core_master_phyconfig_len_write(8);
            spiflash_core_master_phyconfig_width_write(1);
            spiflash_core_master_phyconfig_mask_write(1);
            spiflash_core_master_cs_write(1);

            transfer_byte(0x01);
            transfer_byte(status1);

            spiflash_core_master_cs_write(0);

            while (spiflash_read_status_register() & 1) {
            }
        }
    } else {
        log_printf("SPIFLASH ID invalid, id=0x%08lx", id);
        while (1) {
        }
    }
}
