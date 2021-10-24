# This file is Copyright (c) 2021 Gregory Davill <greg.davill@gmail.com>
# License: BSD

from litex.build.generic_platform import *
from litex.build.lattice import LatticePlatform

# IOs ----------------------------------------------------------------------------------------------

_io = [
    ("clk24", 0, Pins("C1"),  IOStandard("LVCMOS18")),
    ("rst_n", 0, Pins("V17"), IOStandard("LVCMOS33")),

    # Leds
    ("user_led", 0, Pins("N4"), IOStandard("LVCMOS18")),
    
    ("hyper_ram", 0,
        Subsignal("rst_n",     Pins("D2"),       IOStandard("LVCMOS18"),Misc("SLEWRATE=FAST")),    
        Subsignal("clk_p",     Pins("G1"),       IOStandard("LVCMOS18"),Misc("SLEWRATE=FAST")),
        Subsignal("clk_n",     Pins("F2"),       IOStandard("LVCMOS18"),Misc("SLEWRATE=FAST")),
        Subsignal("cs_n",      Pins("F3"),       IOStandard("LVCMOS18"),Misc("SLEWRATE=FAST")),
        Subsignal("dq",        Pins("J2 J1 G4 K1 K4 L1 K3 M1"),     IOStandard("LVCMOS18"),Misc("SLEWRATE=FAST")),
        Subsignal("rwds",      Pins("H1"),       IOStandard("LVCMOS18"),Misc("SLEWRATE=FAST")),
    ),

    ("boson", 0,
        Subsignal("data", Pins("B17 A13 A9 B11 B2 A2 A3 B4 \
                                B1  A8  A7 B9  B8 B6 D1 A6"),IOStandard("LVCMOS18"),Misc("SLEWRATE=SLOW")),
        Subsignal("clk", Pins("A11"),IOStandard("LVCMOS18"),Misc("SLEWRATE=SLOW")),
        Subsignal("vsync", Pins("B10"),IOStandard("LVCMOS18"),Misc("SLEWRATE=SLOW")),
        Subsignal("hsync", Pins("A17"),IOStandard("LVCMOS18"),Misc("SLEWRATE=SLOW")),
        Subsignal("valid", Pins("B18"),IOStandard("LVCMOS18"),Misc("SLEWRATE=SLOW")),
        Subsignal("tx", Pins("B7"),IOStandard("LVCMOS18"),Misc("SLEWRATE=SLOW")),
        Subsignal("rx", Pins("A4"),IOStandard("LVCMOS18"),Misc("SLEWRATE=SLOW")),
        Subsignal("reset", Pins("C2"),IOStandard("LVCMOS18"),Misc("SLEWRATE=SLOW")),
        Subsignal("ext_sync", Pins("A16"),IOStandard("LVCMOS18"),Misc("SLEWRATE=SLOW")),
    ),    

    ("io", 0,
        Subsignal("out", Pins("M2 M3"),IOStandard("LVCMOS18"),Misc("PULLMODE=UP")),
        Subsignal("oe", Pins("N3 N2"),IOStandard("LVCMOS18"),Misc("PULLMODE=UP")),
    ),

    ("sdmmc", 0,
        Subsignal("clk", Pins("J18"),IOStandard("LVCMOS33"),Misc("SLEWRATE=FAST")),
        Subsignal("cmd", Pins("H18"),IOStandard("LVCMOS33"),Misc("PULLMODE=UP"),Misc("SLEWRATE=FAST")),
        Subsignal("data", Pins("K18 L18 F18 G18"),IOStandard("LVCMOS33"),Misc("PULLMODE=UP"),Misc("SLEWRATE=FAST")),
        Subsignal("cd", Pins("F17"),IOStandard("LVCMOS33"),Misc("PULLMODE=UP")),
    ),

    # ("spiflash", 0,
    #     Subsignal("cs_n", Pins("U17"), IOStandard("LVCMOS33")),
    #     #Subsignal("clk",  Pins("U16"), IOStandard("LVCMOS33")), # Note: CLK is bound using USRMCLK block
    #     Subsignal("miso", Pins("T18"), IOStandard("LVCMOS33")),
    #     Subsignal("mosi", Pins("U18"), IOStandard("LVCMOS33")),
    #     Subsignal("wp",   Pins("R18"), IOStandard("LVCMOS33")),
    #     Subsignal("hold", Pins("N18"), IOStandard("LVCMOS33")),
    # ),
    ("spiflash4x", 0,
        Subsignal("cs_n", Pins("U17"), IOStandard("LVCMOS33")),
        #Subsignal("clk",  Pins("U16"), IOStandard("LVCMOS33")),
        Subsignal("dq",   Pins("U18 T18 R18 N18"), IOStandard("LVCMOS33")),
    ),
]



# Connectors ---------------------------------------------------------------------------------------

_connectors = [
]

# Platform -----------------------------------------------------------------------------------------

class Platform(LatticePlatform):
    default_clk_name = "clk48"
    default_clk_period = 1e9/48e6

    def __init__(self, **kwargs):
        LatticePlatform.__init__(self, "LFE5U-25F-8MG285C", _io, _connectors, toolchain='trellis', **kwargs)
        self.name = 'boson_sd'

