#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include <generated/csr.h>

#include "spi.h"


void spiBegin(void) {
	spiflash_core_master_phyconfig_len_write(8);
	spiflash_core_master_phyconfig_width_write(1);
	spiflash_core_master_phyconfig_mask_write(1);
	spiflash_core_master_cs_write(1);
}

void spiEnd(void) {
	spiflash_core_master_cs_write(0);
}

static uint8_t spi_single_xfer(uint8_t b) {
	// wait for tx ready
	while(!spiflash_core_master_status_tx_ready_read()){}
	

	spiflash_core_master_rxtx_write(b);

	//wait for rx ready
	while(!spiflash_core_master_status_rx_ready_read()){}
	

	return spiflash_core_master_rxtx_read();
}

static uint8_t spi_read_status(void) {
	uint8_t val;

	spiBegin();
	spi_single_xfer(0x05);
	val = spi_single_xfer(0);
	spiEnd();
	return val;
}

int spiIsBusy(void) {
  	return spi_read_status() & (1 << 0);
}

__attribute__((used))
uint32_t spiId(void) {
	uint32_t spi_id = 0;

	spiBegin();
	spi_single_xfer(0x90);               // Read manufacturer ID
	spi_single_xfer(0x00);               // Dummy byte 1
	spi_single_xfer(0x00);               // Dummy byte 2
	spi_single_xfer(0x00);               // Dummy byte 3
	spi_id = (spi_id << 8) | spi_single_xfer(0);  // Manufacturer ID
	spi_id = (spi_id << 8) | spi_single_xfer(0);  // Device ID
	spiEnd();

	spiBegin();
	spi_single_xfer(0x9f);               // Read device id
	(void)spi_single_xfer(0);             // Manufacturer ID (again)
	spi_id = (spi_id << 8) | spi_single_xfer(0);  // Memory Type
	spi_id = (spi_id << 8) | spi_single_xfer(0);  // Memory Size
	spiEnd();

	return spi_id;
}

uint8_t spiReset(void) {
	// Writing 0xff eight times is equivalent to exiting QPI mode,
	// or if CFM mode is enabled it will terminate CFM and return
	// to idle.
	unsigned int i;
	spiBegin();
	for (i = 0; i < 8; i++)
		spi_single_xfer(0xff);
	spiEnd();

	// Some SPI parts require this to wake up
	spiBegin();
	spi_single_xfer(0xab);    // Read electronic signature
	spiEnd();

	return 0;
}

int spiInit(void) {
	// Reset the SPI flash, which will return it to SPI mode even
	// if it's in QPI mode, and ensure the chip is accepting commands.
	spiReset();

	spiId();

	return 0;
}


void spiSetQE(void){
	// Check for supported FLASH ID
	uint32_t id = spiId();

    if(id == 0x1f138501){
        // Set QE bit on AT25SF081 if not set
        
		// READ status register
        uint8_t status1 = spi_read_status();

		spiBegin();
		spi_single_xfer(0x35);
		uint8_t status2 = spi_single_xfer(0);
		spiEnd();
        
		// Check Quad Enable bit
        if((status2 & 0x02) == 0){
            // Enable Write-Enable Latch (WEL)
            spiBegin();
            spi_single_xfer(0x06);
            spiEnd();

            // Write back status1 and status2 with QE bit set
            status2 |= 0x02;
            spiBegin();
            spi_single_xfer(0x01);
            spi_single_xfer(status1);
            spi_single_xfer(status2);
            spiEnd();
            
            // loop while write in progress set
            while(spi_read_status() & 1) {}
        }
    }else{
		while(1);
	}
}

void spiFree(void) {

}
