#!/usr/bin/env python3

# This file is Copyright (c) 2020 Gregory Davill <greg.davill@gmail.com>
# License: BSD

# This variable defines all the external programs that this module
# relies on.  lxbuildenv reads this variable in order to ensure
# the build will finish without exiting due to missing third-party
# programs.
LX_DEPENDENCIES = ["riscv", "nextpnr-ecp5", "yosys"]

# Import lxbuildenv to integrate the deps/ directory
import lxbuildenv

import sys
import argparse
import optparse
import subprocess
import os
import shutil
import math

from migen import *

from rtl.platform import boson_frame_grabber_r0d3

from migen.genlib.resetsync import AsyncResetSynchronizer
from litex.build.generic_platform import *
from litex.soc.cores.clock import *
from rtl.ecp5_dynamic_pll import ECP5PLL, period_ns
from litex.soc.integration.soc_core import *
from litex.soc.integration.soc import SoC, SoCRegion
from litex.soc.integration.builder import *

from litex.soc.cores.bitbang import I2CMaster
from litex.soc.cores.gpio import GPIOOut, GPIOIn
from litex.soc.cores import uart

from litex.soc.interconnect import stream, wishbone
from litex.soc.interconnect.wishbone import Interface, Crossbar
from litex.soc.interconnect.csr import *

from litex.soc.cores.led import LedChaser

from migen.genlib.misc import timeline
from migen.genlib.cdc import MultiReg, PulseSynchronizer

from rtl.prbs import PRBSStream
from rtl.wb_streamer import StreamReader, StreamWriter
from rtl.streamable_hyperram import StreamableHyperRAM

from rtl.video.DMACapture import BosonCapture


class _CRG(Module, AutoCSR):

    def __init__(self, platform, sys_clk_freq):
        self.clock_domains.cd_sys = ClockDomain()

        self.clock_domains.cd_hr = ClockDomain()
        self.clock_domains.cd_hr_90 = ClockDomain()
        self.clock_domains.cd_hr2x = ClockDomain()
        self.clock_domains.cd_hr2x_90 = ClockDomain()

        self.clock_domains.cd_init = ClockDomain()

        # # #

        # clk / rst
        clk24 = platform.request("clk24")

        self.submodules.pll = pll = ECP5PLL()
        pll.register_clkin(clk24, 24e6)
        pll.create_clkout(self.cd_hr2x, sys_clk_freq * 2, margin=0)
        pll.create_clkout(self.cd_hr2x_90, sys_clk_freq * 2, phase=1, margin=0)  # SW tunes this phase during init

        #self.comb += self.cd_init.clk.eq(clk24)
        self.comb += self.cd_sys.clk.eq(self.cd_hr.clk)

        platform.add_period_constraint(clk24, period_ns(24e6))

        self._slip_hr2x = CSRStorage()
        self._slip_hr2x90 = CSRStorage()

        # ECLK stuff
        self.specials += [
            Instance("CLKDIVF",
                     p_DIV="2.0",
                     i_ALIGNWD=self._slip_hr2x.storage,
                     i_CLKI=self.cd_hr2x.clk,
                     i_RST=~pll.locked,
                     o_CDIVX=self.cd_hr.clk),
            Instance("CLKDIVF",
                     p_DIV="2.0",
                     i_ALIGNWD=self._slip_hr2x90.storage,
                     i_CLKI=self.cd_hr2x_90.clk,
                     i_RST=~pll.locked,
                     o_CDIVX=self.cd_hr_90.clk),
            AsyncResetSynchronizer(self.cd_hr2x, ~pll.locked),
            AsyncResetSynchronizer(self.cd_hr, ~pll.locked),
            AsyncResetSynchronizer(self.cd_hr2x_90, ~pll.locked),
            AsyncResetSynchronizer(self.cd_hr_90, ~pll.locked),
            AsyncResetSynchronizer(self.cd_sys, ~pll.locked),
        ]

        self._phase_sel = CSRStorage(2)
        self._phase_dir = CSRStorage()
        self._phase_step = CSRStorage()
        self._phase_load = CSRStorage()

        self.comb += [
            self.pll.phase_sel.eq(self._phase_sel.storage),
            self.pll.phase_dir.eq(self._phase_dir.storage),
            self.pll.phase_step.eq(self._phase_step.storage),
            self.pll.phase_load.eq(self._phase_load.storage),
        ]


class Boson_SoC(SoCCore):
    csr_map = {}
    csr_map.update(SoCCore.csr_map)

    mem_map = {
        **SoCCore.mem_map,
        **{
            "sram":           0x10000000,
            "csr":            0xf0000000,
            "vexriscv_debug": 0xf00f0000,
            "hyperram":       0x20000000,
            "spiflash":       0x30000000,
        },
    }

    interrupt_map = {}
    interrupt_map.update(SoCCore.interrupt_map)

    def __init__(self, target):

        self.platform = platform = boson_frame_grabber_r0d3.Platform()
        self.target = target

        platform.sys_clk_freq = sys_clk_freq = 70e6
        SoCCore.__init__(
            self,
            platform,
            clk_freq=sys_clk_freq,
            cpu_type='vexriscv',
            cpu_variant='standard',
            with_uart=False,
            csr_data_width=32,
            ident_version=False,
            wishbone_timeout_cycles=1024*8,
            integrated_sram_size=(64 if target == 'main' else 96) * 1024,
            cpu_reset_address=self.mem_map['sram'],
        )

        # Toolchain config
        platform.toolchain.build_template[0] = "yosys -q -l {build_name}.rpt {build_name}.ys"
        platform.toolchain.build_template[1] += f" --log {platform.name}-nextpnr.log --router router1"
        platform.toolchain.build_template[1] += f" --report {platform.name}-timing.json"
        platform.toolchain.yosys_template.insert(-1, "scratchpad -copy abc9.script.flow3 abc9.script")
        platform.toolchain.yosys_template.insert(-1, "scratchpad -set abc9.D {}".format(int((1e12/sys_clk_freq)/2)))
        platform.toolchain.yosys_template[-1] += ' -abc9'  # abc2/nowidelut generally give higher freq

        self.platform.name = "boson_sd_{}".format(target)

        # crg -------------------------------------------------------------------------------------
        self.submodules.crg = _CRG(platform, sys_clk_freq)

        # Timers -----------------------------------------------------------------------------------
        self.add_timer(name="timer1")
        self.add_timer(name="timer2")

        # Leds -------------------------------------------------------------------------------------
        leds = platform.request_all("user_led")
        led_pad = Signal(1)
        self.submodules.leds = LedChaser(pads=leds, sys_clk_freq=sys_clk_freq)

        # SPI Flash --------------------------------------------------------------------------------
        from litespi.modules import MX25R1635F
        from litespi.opcodes import SpiNorFlashOpCodes as Codes
        self.add_spi_flash(mode="4x", module=MX25R1635F(Codes.READ_1_1_4), with_master=True)

        # HyperRAM with DMAs -----------------------------------------------------------------------
        self.submodules.writer = writer = StreamWriter()
        self.submodules.reader = reader = StreamReader()
        self.submodules.hyperram = hyperram = StreamableHyperRAM(platform.request("hyper_ram"), devices=[reader, writer])
        self.register_mem("hyperram", self.mem_map['hyperram'], hyperram.bus, size=0x800000)

        # PRBS Tester, used to test HyperRAM DMAs --------------------------------------------------
        self.submodules.prbs = PRBSStream()
        reader.add_source(self.prbs.source.source, "prbs")
        writer.add_sink(self.prbs.sink.sink, "prbs")

        # SDMMC ------------------------------------------------------------------------------------
        self.add_sdcard(name="sdmmc", mode="read+write", use_emulator=False, software_debug=False)

        # Boson -----------------------------------------------------------------------------------
        self.submodules.boson = BosonCapture(platform, reader)
        
        # IO/UART ----------------------------------------------------------------------------------
        io = platform.request("io")
        self.submodules.io_oe = GPIOOut(io.oe)
        uart_pads = uart.UARTPads()

        self.submodules.uart_phy = uart.UARTPHY(
            pads     = uart_pads,
            clk_freq = self.sys_clk_freq,
            baudrate = 1000000)
        self.submodules.uart = uart.UART(self.uart_phy,
            tx_fifo_depth = 8,
            rx_fifo_depth = 8)

        self.irq.add("uart", use_loc_if_exists=True)

        # Tristate control on USER I/O, connect UART.
        io_out = [TSTriple(), TSTriple()]
        self.specials += io_out[0].get_tristate(io.out[0])
        self.specials += io_out[1].get_tristate(io.out[1])
        self.comb += [
            io_out[0].o.eq(uart_pads.tx),
            io_out[1].o.eq(0),

            uart_pads.rx.eq(io_out[1].i),
            io_out[0].oe.eq(io.oe[0]),
            io_out[1].oe.eq(io.oe[1]),
        ]
        
        # SW controlled Reset -------------------------------------------------------------------------
        rst = Signal()
        self.submodules.reset = GPIOOut(rst)
        self.comb += platform.request("rst_n").eq(~rst)        

        #Add GIT repo to the firmware
        git_rev_cmd = subprocess.Popen("git describe --tags --first-parent --always".split(),
                        stdout=subprocess.PIPE,
                        stderr=subprocess.PIPE)
        (git_stdout, _) = git_rev_cmd.communicate()
        self.add_constant('CONFIG_REPO_GIT_DESC',git_stdout.decode('ascii').strip('\n'))

    # Add SDCard -----------------------------------------------------------------------------------
    def add_sdcard(self, name="sdcard", mode="read+write", use_emulator=False, software_debug=False):
        # Imports.
        from litesdcard.emulator import SDEmulator
        from litesdcard.phy import SDPHY
        from litesdcard.core import SDCore
        from litesdcard.frontend.dma import SDBlock2MemDMA
        from rtl.litesdcard_dma import SDMem2BlockDMA

        # Checks.
        assert mode in ["read", "write", "read+write"]

        # Emulator / Pads.
        if use_emulator:
            sdemulator = SDEmulator(self.platform)
            self.submodules += sdemulator
            sdcard_pads = sdemulator.pads
        else:
            sdcard_pads = self.platform.request(name)

        # Core.
        self.check_if_exists("sdphy")
        self.check_if_exists("sdcore")
        self.submodules.sdphy  = SDPHY(sdcard_pads, self.platform.device, self.clk_freq, cmd_timeout=500e-3, data_timeout=500e-3)
        self.submodules.sdcore = SDCore(self.sdphy)

        # Block2Mem DMA.
        if "read" in mode:
            bus = wishbone.Interface(data_width=self.bus.data_width, adr_width=self.bus.address_width)
            self.submodules.sdblock2mem = SDBlock2MemDMA(bus=bus, endianness=self.cpu.endianness)
            self.comb += self.sdcore.source.connect(self.sdblock2mem.sink)
            dma_bus = self.bus if not hasattr(self, "dma_bus") else self.dma_bus
            dma_bus.add_master("sdblock2mem", master=bus)

        # Mem2Block DMA.
        if "write" in mode:
            bus = wishbone.Interface(data_width=self.bus.data_width, adr_width=self.bus.address_width)
            self.submodules.sdmem2block = SDMem2BlockDMA(bus=bus, endianness=self.cpu.endianness, writer=self.writer)
            self.comb += self.sdmem2block.source.connect(self.sdcore.sink)
            dma_bus = self.bus if not hasattr(self, "dma_bus") else self.dma_bus
            dma_bus.add_master("sdmem2block", master=bus)

        

    def do_exit(self, vns):
        if hasattr(self, "analyzer"):
            self.analyzer.export_csv(vns, "test/analyzer.csv")

    def PackageFirmware(self, builder):
        # Remove un-needed sw packages
        builder.software_packages = []
        builder.add_software_package("libc")
        builder.add_software_package("libcompiler_rt")
        builder.add_software_package("libbase")

        if self.target == 'main':
            builder.add_software_package("main_fw", "{}/../firmware/main_fw".format(os.getcwd()))
        elif self.target == 'boot':
            builder.compile_software = False
            return


        builder._prepare_rom_software()
        builder._generate_includes()
        builder._generate_rom_software(compile_bios=False)

        # lock out compiling firmware during build steps
        builder.compile_software = False

    def PackageBooter(self, builder):
        self.finalize()

        os.makedirs(builder.output_dir, exist_ok=True)

        # Remove un-needed sw packages
        builder.software_packages = []
        builder.add_software_package("libc")
        builder.add_software_package("libbase")
        builder.add_software_package("libcompiler_rt")

        if self.target == 'main':
            builder.add_software_package("main_fw_bootstrap", "{}/../firmware/main_fw_bootstrap".format(os.getcwd()))
        elif self.target == 'boot':
            builder.add_software_package("bootloader", "{}/../firmware/bootloader".format(os.getcwd()))

        builder._prepare_rom_software()
        builder._generate_includes()
        builder._generate_rom_software(compile_bios=False)


def CreateFirmwareInit(init, output_file):
    content = ""
    for d in init:
        content += "{:08x}\n".format(d)
    with open(output_file, "w") as o:
        o.write(content)


def main():
    parser = argparse.ArgumentParser(description="Build Boson SD Gateware")
    parser.add_argument("--update-firmware",
                        default=False,
                        action='store_true',
                        help="compile firmware and update existing gateware")
    parser.add_argument("--target",
                        default='main',
                        help="Select which SoC to build [boot, main]")
    
    args = parser.parse_args()

    soc = Boson_SoC(args.target)
    builder = Builder(soc, output_dir="build", csr_csv="build/csr.csv")

    # Check if we have the correct files
    firmware_file = os.path.join(builder.output_dir, "software", "main-fw", "main-fw.bin")

    rand_rom = os.path.join(builder.output_dir, "gateware", "rand.data")

    input_config = os.path.join(builder.output_dir, "gateware", f"{soc.platform.name}.config")
    output_config = os.path.join(builder.output_dir, "gateware", f"{soc.platform.name}_patched.config")

    # Create 256bytes rand fill for BRAM
    if (os.path.exists(rand_rom) == False) or (args.update_firmware == False):
        os.makedirs(os.path.join(builder.output_dir, 'software'), exist_ok=True)
        os.makedirs(os.path.join(builder.output_dir, 'gateware'), exist_ok=True)
        os.system(f"ecpbram  --generate {rand_rom} --seed {0} --width {32} --depth {soc.integrated_sram_size // 4}")

        # patch random file into BRAM
        data = []
        with open(rand_rom, 'r') as inp:
            for d in inp.readlines():
                data += [int(d, 16)]
        soc.sram.mem.init = data

        # Build gateware
        vns = builder.build()
        soc.do_exit(vns)

    gateware_offset = 0x00080000

    if args.target == 'main':
        # create a compressed bitstream
        output_bit = os.path.join(builder.output_dir, "gateware", "boson_sd_main_bitstream.bit")
        os.system(f"ecppack --freq 38.8 --compress --input {input_config} --bit {output_bit}")

        # Determine Bitstream size
        stage_1_filesize = os.path.getsize(output_bit) + 8 # Add 8 for magic/crc
        alignment = 4 - 1 # Pad out to next word address
        firmware_offset = (stage_1_filesize + alignment) & ~(alignment)  # Add padding, align FW to next block
        firmware_offset += gateware_offset  # bitstream offset
        print(f"Compressed file size: 0x{stage_1_filesize:0x}")
        print(f"Placing firmware at: 0x{firmware_offset:0x}")

        # Finalise the gateware aspects of the design.
        # Alter the spiflash origin so that our actual address is valid in the linker when compiling
        # Compile booter, it makes use of SPIFLASH_BASE for the boot address
        soc.finalize()
        soc.mem_regions['spiflash'].origin += firmware_offset
    else:
        output_bit = os.path.join(builder.output_dir, "gateware", "boson_sd_bootloader_bitstream.bit")

    soc.PackageBooter(builder)

    if args.target == 'main':
        blkram_file = "{}/software/main_fw_bootstrap/main_fw_bootstrap.bin".format(builder.output_dir)
    elif args.target == 'boot':
        blkram_file = "{}/software/bootloader/bootloader.bin".format(builder.output_dir)
    
    blkram_init = "{}/gateware/blkram_fw.init".format(builder.output_dir)
    CreateFirmwareInit(get_mem_data(blkram_file, soc.cpu.endianness), blkram_init)

    # Insert Firmware into Gateware
    os.system(f"ecpbram  --input {input_config} --output {output_config} --from {rand_rom} --to {blkram_init}")

    # create a compressed bitstream
    os.system(f"ecppack --freq 38.8 --compress {'--bootaddr 0x{:x}'.format(gateware_offset) if args.target == 'boot' else ''} --input {output_config} --bit {output_bit}")

    if args.target == 'main':
        
        # Build firmware
        soc.PackageFirmware(builder)

        # Combine FLASH firmware
        from util.combine import CombineBinaryFiles
        flash_regions_final = {
            "build/gateware/boson_sd_main_bitstream.bit": gateware_offset,   # SoC ECP5 Bitstream
            "build/software/main_fw/main_fw.bin": firmware_offset,  # main firmware
        }
        output_bin = os.path.join(builder.output_dir, "gateware", "boson_sd_main.bin")
        CombineBinaryFiles(flash_regions_final, output_bin)

        
        print(f"""Boson SD build complete!  
        
    boson_sd_main.bin 
        size={os.path.getsize(output_bin) / 1024 :.2f}KB ({os.path.getsize(output_bin)} bytes) 
        FLASH Usage: {(float)(os.path.getsize(output_bin)) / (((2*1024*1024) - gateware_offset)/100) :.2f} %
        """)
    else:
        print(f"""Boson SD bootloader build complete!  
        
    boson_sd_bootloader_bitstream.bit size={os.path.getsize(output_bit) / 1024 :.2f}KB ({os.path.getsize(output_bit)} bytes) 
        FLASH Sector Usage: {math.ceil(os.path.getsize(output_bit) / (64*1024)) :} / {1024 // 64}
        """)


if __name__ == "__main__":
    main()
