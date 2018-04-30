#!/usr/bin/env python3

import sys

from litex.build.generic_platform import *
from litex.build.lattice import LatticePlatform

from vgen.soc.cores.lattice.ice40 import CRG,RGB,SB_SPRAM256K,SB_SPI,SB_LEDDA
from vgen.soc.cores.misc import Sequencer,Version



from migen import *
from litex.soc.interconnect import stream
from litex.soc.interconnect.csr import *
from litex.soc.interconnect.csr_bus import *
from litex.soc.interconnect import wishbone

from litex.soc.cores.uart import UARTWishboneBridge

from litex.soc.integration.soc_core import *
from litex.soc.integration.builder import *

from migen.genlib.misc import timeline


class BaseSoC(SoCCore):
    csr_map = {
            "spi0"    : 16,
            "spi1"    : 17,
            "version" : 18


    }
    csr_map.update(SoCCore.csr_map)

    mem_map = {
                "spi0"  : 0x80000000,                        #64K address fir each spi (support memory)
                "spi1"  : 0x90000000
    }
    mem_map.update(SoCCore.mem_map)
    
    def __init__(self, stl, with_cpu=False,sim=False,sys_clk_freq=12e6,baudrate=115200,sim=False):
        platform = stl.platform
        sys_clk_freq = int(sys_clk_freq)

        integrated_rom_size = 0
        integrated_rom_init = []
        if with_cpu:
            integrated_rom_size = 0x2000
            integrated_rom_init = get_firmware_data("./firmware/firmware.bin", 0x2000)

        SoCCore.__init__(self, platform,
            clk_freq=sys_clk_freq,
            cpu_type="riscv32" if with_cpu else None,
            csr_data_width=8,
            with_uart=with_cpu, uart_baudrate=9600,
            with_timer=with_cpu,
            ident="PicoSoC",
            ident_version=True,
            integrated_rom_size=integrated_rom_size,
            integrated_rom_init=integrated_rom_init)

        #Configure oscillator and clock
        self.submodules.osc = OSC(freq="{}MHz".format(int(sys_clk_freq/1e6)),sim=sim)
        if not sim:
            self.submodules.crg = CRG(clk = self.osc.clk)

        # bridge
        if not with_cpu:
            self.add_cpu_or_bridge(UARTWishboneBridge(platform.request("serial"), sys_clk_freq, baudrate=baudrate))
            self.add_wb_master(self.cpu_or_bridge.wishbone)
        

        #RGB LED Output
        self.submodules.ledc=RGB(pads=platform.request("led"))

        #Blinker RGB
        self.submodules.sequencer = Sequencer(clk_freq=sys_clk_freq,duty_cycle=0.1,period=1,N=6)

        
        self.comb+= [
                        self.ledc.RGBPWM0.eq(self.sequencer.output[2]),
                        self.ledc.RGBPWM1.eq(self.sequencer.output[1]),
                        self.ledc.RGBPWM2.eq(self.sequencer.output[0])
            ]

        self.comb+= [   self.ledc.RGBLEDEN.eq(1),
                        self.ledc.CURREN.eq(1)
        ]


        #Version for code
        self.submodules.version=Version(0xaa)
        
        
        

      def __init__(self, with_cpu=True):
        platform = Platform()
        sys_clk_freq = int(16e6)

        integrated_rom_size = 0
        integrated_rom_init = []
        if with_cpu:
            integrated_rom_size = 0x2000
            integrated_rom_init = get_firmware_data("./firmware/firmware.bin", 0x2000)

        SoCCore.__init__(self, platform,
            clk_freq=sys_clk_freq,
            cpu_type="lm32" if with_cpu else None,
            csr_data_width=8,
            with_uart=with_cpu, uart_baudrate=9600,
            with_timer=with_cpu,
            ident="TinyFPGA Test SoC",
            ident_version=True,
            integrated_rom_size=integrated_rom_size,
            integrated_rom_init=integrated_rom_init)

        self.submodules.crg = _CRG(platform)

        # bridge
        if not with_cpu:
            self.add_cpu_or_bridge(UARTWishboneBridge(platform.request("serial"), sys_clk_freq, baudrate=9600))
            self.add_wb_master(self.cpu_or_bridge.wishbone)

        led_counter = Signal(32)
        self.sync += led_counter.eq(led_counter + 1)
        self.comb += [
            platform.request("user_led", 0).eq(led_counter[22]),
            platform.request("user_led", 1).eq(led_counter[23]),
            platform.request("user_led", 2).eq(led_counter[24])
        ]
