from migen import *
from migen.genlib.io import CRG
from migen.fhdl import verilog
from ice40 import OSC,SB_SPI
from upduino_v2 import Platform


class Blinker(Module):
        def __init__(self,pad=None,clk_freq=12e6,period = 0.5):
            self.counter = Signal(max= int(clk_freq*period)+1,reset=0)
            if pad is None:
                self.pad = Signal()
            else:
                self.pad = pad
            self.sync +=  self.counter.eq(self.counter+1)
            self.comb +=  self.pad.eq(self.counter[-1])



class SPIDEMO(Module):
    def __init__(self,platform=None,pads_led = None,sim=False):
        spi_regs = {"SPICR0"  :0x08,
            "SPICR1"  :0x09,
            "SPICR2"  :0x0A,
            "SPIBR"   :0x0B,
            "SPITXDR" :0x0D,
            "SPIRXDR" :0x0E,
            "SPICSR"  :0x0F,
            "SPISR"   :0x0C,
            "SPIINTSR":0x06,
            "SPIINTCR":0x07,
            "END"     :0x10
        }

        spi_init = [("SPICR1",0x80),     #Reset SPI
                    ("SPIBR", 0x3f),     #Set Divider
                    ("SPICR2",0xC0),     #Set Master Mode
                    ("END",0xAA)
                    ]
        
        spi_test_str = b'Hello World\r'

        #Create and initialize spi init sequence memory    addr[4..0]data[7..0]
        self.specials.mem_spi_init = Memory(width=13,depth=len(spi_init),init=[spi_regs[e[0]] <<8 | e[1] for e in spi_init])
        #Crete and initialize spi test string memory
        self.specials.mem_test_data = Memory(width=8,depth=len(spi_test_str),init=[char for char in spi_test_str])

        self.submodules.osc = OSC(freq="12MHz",sim=sim)
        self.submodules.crg = CRG(clk = self.osc.clk)
        self.submodules.blinker = Blinker(platform.request("led"),clk_freq=self.osc.frequency,period=0.5)
        self.submodules.spi0 = SB_SPI(pads = platform.request("spi"),sim=sim)

    def sim_memory_test(self):
        print("Reading SPI Initialization Memory")
        for i in range(self.mem_spi_init.depth):
            value = yield self.mem_spi_init[i]
            self.display_mem(i,value)
        yield
        yield
        for i in range(self.mem_test_data.depth):
            value = yield self.mem_test_data[i]
            self.display_mem(i,value)
        yield
        yield

    def display_mem(self,i,value):
         print("mem_spi_init[{}] --> \t Addr = {:02x}, Data {:02x}".format(i,(value >> 8) & 0x1f ,value & 0xff))

    def run_simulation(self):
        run_simulation(self,self.sim_memory_test(),vcd_name="spi_test.vcd")




if __name__ == "__main__":
    plat = Platform()
    spidemo = SPIDEMO(platform=plat,sim=True)
    print(spidemo.osc.frequency)
    spidemo.run_simulation()
    #plat.build(spidemo)
    #plat.create_programmer().flash(address=0,bitstream_file="build/top.bin")
    
    




