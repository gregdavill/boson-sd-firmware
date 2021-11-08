
import sys
import argparse
import optparse
import subprocess
import os
import shutil

from migen import *
from migen.genlib.misc import timeline


from litex.build.generic_platform import *
from litex.soc.cores.clock import *
from rtl.ecp5_dynamic_pll import ECP5PLL, period_ns
from litex.soc.integration.soc_core import *
from litex.soc.integration.soc import SoCRegion
from litex.soc.integration.builder import *

from litex.soc.interconnect.stream import Pipeline, StrideConverter, Endpoint, AsyncFIFO, Gate
from litex.soc.interconnect.csr import *

from migen.genlib.cdc import PulseSynchronizer

from rtl.video.boson import Boson
from rtl.video.framer import FrameExtraction

from litevideo.input.analysis import  ResolutionDetection



class FrameExtraction(Module, AutoCSR):
    def __init__(self):
        # in pix clock domain
        self.vsync = Signal()

        self.sof = Signal() 
        # # #

        # start of frame detection
        vsync = self.vsync
        vsync_r = Signal()
        self.new_frame = new_frame = Signal()
        self.comb += new_frame.eq(vsync & ~vsync_r)
        self.sync.pix += vsync_r.eq(vsync)

        ps = PulseSynchronizer("pix", "sys")
        self.comb += ps.i.eq(new_frame), self.sof.eq(ps.o)
        self.submodules += ps






class BosonCapture(Module, AutoCSR):
    def __init__(self, platform, reader):

        start = Signal()

        # Boson video stream
        self.submodules.boson = boson = Boson(platform, platform.request("boson"), platform.sys_clk_freq)
        self.submodules.resolution = res = ResolutionDetection()        
        fe = FrameExtraction()
        self.submodules += fe


        self.comb += [
            res.valid_i.eq(1),
            res.vsync.eq(boson.vsync),
            res.de.eq(boson.data_valid),

            fe.vsync.eq(boson.vsync),
        ]

        # 16bit -> 32bit coverter
        sc = StrideConverter([('data', 16)], [('data', 32)], reverse=False)
        sc = ResetInserter()(sc)
        sc = ClockDomainsRenamer({"sys": "pix"})(sc)
        self.submodules += sc

        # FIFO
        fifo = AsyncFIFO([('data', 32)], 512, buffered=True)
        fifo = ClockDomainsRenamer({"write": "pix", "read": "sys"})(fifo)
        fifo = ResetInserter(["pix", "sys"])(fifo)
        self.submodules += fifo

        gate = Gate([('data', 32)], True)
        self.submodules += gate

        self.submodules.pipeline = pipeline = Pipeline(boson, sc, fifo, gate)

        self.sync += timeline(fe.sof, 
            [
                (50, [start.eq(1)]),
                (51, [start.eq(0)]),
            ],
        )

        enabled = Signal()
        self.sync += [
            If(fe.sof & reader.enabled,
                enabled.eq(1),
            ).Elif(~reader.enabled,
                enabled.eq(0)
            )
        ]

        self.comb += [
            gate.enable.eq(enabled)
        ]

        reader.add_source(pipeline.source, "boson", start)


