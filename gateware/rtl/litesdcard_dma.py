#
# This file is based on the file from LiteSDCard.
#
# Copyright (c) 2020 Florent Kermarrec <florent@enjoy-digital.fr>
# Copyright (c) 2021 Greg Davill    <greg.davill@gmail.com>
# SPDX-License-Identifier: BSD-2-Clause

from migen import *

from litex.soc.interconnect.csr import *
from litex.soc.interconnect import stream

from litex.soc.cores.dma import WishboneDMAReader, WishboneDMAWriter

class SDMem2BlockDMA(Module, AutoCSR):
    """Memory to Block DMA

    Read data from memory through DMA and generate a stream of blocks.
    """
    def __init__(self, bus, endianness, fifo_depth=512, writer=None):
        self.bus    = bus
        self.source = stream.Endpoint([("data", 8)])
        self.irq    = Signal()

        # # #

        # Submodules
        self.submodules.dma = WishboneDMAReader(bus, with_csr=True, endianness=endianness)
        converter = stream.Converter(bus.data_width, 8, reverse=True)
        fifo      = stream.SyncFIFO([("data", 8)], fifo_depth, buffered=True)
        self.submodules += converter, fifo


        fifo0      = stream.SyncFIFO([("data", 32)], 128, buffered=True)
        fifo0      = ResetInserter()(fifo0)
        converter0 = stream.Converter(32, 8, reverse=False)
        converter0 = ResetInserter()(converter0)
        self.submodules += converter0, fifo0

        if writer:
            writer.add_sink(fifo0.sink, "SD")

        
        # Flow
        self.comb += [
            self.dma.source.connect(converter.sink),
            converter.source.connect(fifo.sink),

            fifo0.source.connect(converter0.sink),
            If(writer.enabled,
                converter0.source.connect(self.source),
            ).Else(
                fifo0.reset.eq(1),
                converter0.reset.eq(1),
                fifo.source.connect(self.source),
            ),
        ]

        # Block delimiter
        count = Signal(9)
        self.sync += [
            If(self.source.valid & self.source.ready,
                count.eq(count + 1),
                If(self.source.last, count.eq(0))
            )
        ]
        self.comb += If(count == (512 - 1), self.source.last.eq(1))

        # IRQ / Generate IRQ on DMA done rising edge
        done_d = Signal()
        self.sync += done_d.eq(self.dma._done.status)
        self.sync += self.irq.eq(self.dma._done.status & ~done_d)
