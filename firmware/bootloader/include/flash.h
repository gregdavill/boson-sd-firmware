/*
 *  Copyright 2021 Gregory Davill <greg.davill@gmail.com>
 */
#ifndef FLASH_H_
#define FLASH_H_

#include <stdbool.h>
#include <stdint.h>

void spiflash_read_uuid(uint8_t* uuid);

int spiflash_write_stream(uint32_t addr, uint8_t* stream, int len);
void spiflash_read_uuid(uint8_t* uuid);
bool spiflash_protection_read(void);
void spiflash_protection_write(bool lock);
uint32_t spiflash_read_status_register(void);
void spiflash_write_enable(void);

void spiflash_page_program(uint32_t addr, uint8_t *data, int len);
void spiflash_sector_erase(uint32_t addr);

#define FLASH_64K_BLOCK_ERASE_SIZE (64 * 1024)
#define FLASH_4K_BLOCK_ERASE_SIZE (4 * 1024)

#endif /* FLASH_H_ */
