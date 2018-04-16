from migen.build.generic_platform import *
from migen.build.lattice import LatticePlatform
from migen.build.lattice.programmer import IceStormProgrammer

_io = [
    ("led", 0, Pins("12"), IOStandard("LVCMOS33")),

    ("ledrgb", 0,
        Subsignal("b",  Pins("39")),
        Subsignal("g",  Pins("40")),
        Subsignal("r",  Pins("41"))
    ),

    ("serial", 0,
        Subsignal("tx", Pins("38")),      #JP5-15
        Subsignal("rx", Pins("28")),      #JP5-16
        IOStandard("LVCMOS33")          
    ),

    ("spi", 0,
        Subsignal("ssn",  Pins("34")),     #JP5-11
        Subsignal("sck",  Pins("43")),     #JP5-12
        Subsignal("mosi", Pins("36")),     #JP5-13      
        Subsignal("miso", Pins("42")),     #JP5-14
    ),

    ("spiflash", 0,
        Subsignal("cs_n", Pins("16"), IOStandard("LVCMOS33")),
        Subsignal("clk",  Pins("15"), IOStandard("LVCMOS33")),
        Subsignal("mosi", Pins("14"), IOStandard("LVCMOS33")),
        Subsignal("miso", Pins("17"), IOStandard("LVCMOS33"))
    ),

    ("test", 0, 
        Subsignal("p0", Pins("31")), 
        Subsignal("p1", Pins("32")),
        Subsignal("p2", Pins("27")),
        Subsignal("p3", Pins("26")),
        IOStandard("LVCMOS33"))

]

_connectors = [
    #Pin 3-16, Pin 8 -> IOT_46B_G0(35), Pin  10 -> IOT_45A_G1(37)
    ("JP5", "23 25 26 27 32 35 31 37"),  #SPI(34 43 36 42)  UART(38,28
    #Pin 1-16 Pin 9 -> IOB_3B_G6(44)
    ("JP6", "12 21 13 19 18 11 10 9 6 44 4 3 48 45 47 44 46 2"),  
    #TP4 -> IOB_25B_G3 (20)
    ("TP","20 10")
]


class Platform(LatticePlatform):
    def __init__(self):
        LatticePlatform.__init__(self, "ice40-up5k-sg48", _io, _connectors,
                                 toolchain="icestorm")

    def create_programmer(self):
        return IceStormProgrammer()
        