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
    spi_regs = {
            "SPICR0"  :0x08,
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

    def __init__(self,platform=None,pads_led = None,sim=False):
        spi_init = [("SPICR1",0x80),     #Reset SPI
                    ("SPIBR", 0x3f),     #Set Divider
                    ("SPICR2",0xC0),     #Set Master Mode
                    ("END",0xAA)
                    ]
        
        spi_test_str = b'Hello World\r'

        #Create and initialize spi init sequence memory    addr[4..0]data[7..0]
        mem_depth = len(spi_init) + len(spi_test_str) + 1   #include terminate value
        mem_width = 12   #4 bit Addr, 8 bit Data
        mem_i_config   =  [(SPIDEMO.spi_regs[e[0]] <<8 | e[1])       for e in spi_init]
        mem_i_test_str =  [(SPIDEMO.spi_regs["SPITXDR"] << 8 | char) for char in spi_test_str]
        self.specials.mem = Memory(width=mem_width,depth=mem_depth,init=mem_i_config + mem_i_test_str)

        self.submodules.osc = OSC(freq="12MHz",sim=sim)
        self.submodules.crg = CRG(clk = self.osc.clk)
        self.submodules.blinker = Blinker(platform.request("led"),clk_freq=self.osc.frequency,period=0.5)
        self.submodules.spi0 = SB_SPI(pads = platform.request("spi"),sim=sim)

        index = Signal(8,reset=0)

        ctrlfsm = FSM()
        self.submodules += ctrlfsm

        ctrlfsm.act("START",
            NextState("SEND_INIT")
        )

        ctrlfsm.act("WRITE_CONFIG_DATA",
            self.tb_i.eq(1),
            self.rw_i.eq(1),
            self.data_i.eq(self.mem[index][:8]),    
            self.addr_i.eq(self.mem[index][8:]),
            NextValue(index,index + 1),
            NextState("WAIT_WRITE_CONFIG_ACK")
        )

        ctrlfsm.act("WAIT_WRITE_CONFIG_ACK",
            If(self.spi.ack_o == 0,
                If (index == len(spi_init),
                    self.tb.i.eq(0),
                    NextState("WRITE_TEST_DATA")
                )
            )
        )

        ctrlfsm.act("WRITE_TEST_DATA",
            self.tb_i.eq(1),
            self.rw_i.eq(1),
            self.data_i.eq(self.mem[index][:8]),    
            self.addr_i.eq(self.mem[index][8:]),
            NextValue(index,index + 1),
            NextState("WAIT_WRITE_TEST_DATA_ACK")
        )

        ctrlfsm.act("WAIT_WRITE_TEST_DATA_ACK",
            If(self.spi.ack_o == 0,
                If (index == len(spi_init),
                    self.tb.i.eq(0),
                    NextState("READY_WRITE_TEST_DATA") #BEN WAS HERE
            )
        




    
    def tb_spi_demo(self):
        print("Reading SPI Initialization Memory")
        for i in range(self.mem.depth):
            value = yield self.mem[i]
            addr = (value >> 8) & 0x1f
            data = value & 0xff
            print("mem_spi_init[{}] --> \t Addr = {:02x}, Data {:02x}".format(i,addr ,data),end="")
            if addr == SPIDEMO.spi_regs["SPITXDR"]:
                if (data != ord('\r')):
                    print("[{:c}]".format(data),end="")
                else:
                    print("[<newline>]",end="")
            print()   
        yield
        yield

    def run_simulation(self):
        run_simulation(self,self.tb_spi_demo(),vcd_name="spi_test.vcd")




if __name__ == "__main__":
    plat = Platform()
    spidemo = SPIDEMO(platform=plat,sim=True)
    print(spidemo.osc.frequency)
    spidemo.run_simulation()
    #plat.build(spidemo)
    #plat.create_programmer().flash(address=0,bitstream_file="build/top.bin")
    
    




