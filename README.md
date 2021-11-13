# Boson SD Firmware

This project contains firmware and gateware required to run the boson frame grabber PCB. It enables capturing of RAW data from the FLIR boson camera core to an SD card.

## Directory Structure

 - firmware: firmware that runs on a softcore RISC-V
 - gateware: gateware that describes the hardware inside the FPGA


## Bootloader

The bootloader is a standalone SoC that includes a firmware embedded in FPGA blockram, the bitstream for this is loaded at `0x00000` in FLASH.
The ECP5 will load this bitstream first after a POR.

The bootloader makes use of the USER I/O as a serial UART. 
Over this UART the bootloader outputs debug/status info, it is recommended to connect a serial console when performing a firmware upgrade.

The bootloader bitstream has a `bootaddr` value set to `0x80000`. When it triggers the ECP5 reconfiguration cycle the ECP5 will load a bitstream from `0x80000`, which is where we place the main user bitstream.

### Upgrading the bootloader

The bootloader FLASH sectors are locked, to ensure no accidental writes cause corruption. However the bootloader does support unlocking it's FLASH sectors `0x00000`-`0x80000`. 
This is achieved using a json configuration file instead of just a standard binary file.

Here is an example of a json file that will unlock the bootloader sectors and then load a binary at `0x00000`.

The file must be named `boot.json`

```json
{
	"bootloader_protection": "disable",
	"boson_sd_bootloader.bin": "0x00000000"
}
```

The bootloader will re-lock these sectors on exiting.

Note: If the bootloader is corrupted then a JTAG connection will need to be made to the test-pad on the PCB, and the program `ecpprog` used to re-flash a working bootloader.