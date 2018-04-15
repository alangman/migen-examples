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
        
        spi_test_str = b'Hello World, And Welcome\r'

        #Create and initialize spi init sequence memory    addr[4..0]data[7..0]
        mem_depth = len(spi_init) + len(spi_test_str) + 1   #include terminate value
        mem_width = 12   #4 bit Addr, 8 bit Data
        mem_i_config   =  [(SPIDEMO.spi_regs[e[0]] <<8 | e[1])       for e in spi_init]
        mem_i_test_str =  [(SPIDEMO.spi_regs["SPITXDR"] << 8 | char) for char in spi_test_str]
        self.specials.mem = Memory(width=mem_width,depth=mem_depth,init=mem_i_config + mem_i_test_str)
        self.specials.p_mem = self.mem.get_port(write_capable=False)
        self.submodules.osc = OSC(freq="12MHz",sim=sim)
        self.clk = self.osc.clk
        self.submodules.crg = CRG(clk = self.clk)
        self.submodules.blinker = Blinker(platform.request("led"),clk_freq=self.osc.frequency,period=0.5)
        self.submodules.spi = SB_SPI(pads = platform.request("spi"),sim=sim)

        _index = Signal(8,reset=0)   #Make sure the width of index is the same as the addr of the mem
        self.index = _index
        self.submodules.ctrlfsm  = FSM()
        ctrlfsm = self.ctrlfsm

        ctrlfsm.act("START",
            #add delay other setup.
            self.p_mem.adr.eq(_index),
            NextState("WRITE_CONFIG_DATA")
        )

        ctrlfsm.act("WRITE_CONFIG_DATA",
            self.spi.tb_i.eq(1),
            self.spi.rw_i.eq(1),
            self.spi.dat_i.eq(self.p_mem.dat_r[:8]),    
            self.spi.addr_i.eq(self.p_mem.dat_r[8:]),
            NextValue(_index,_index + 1),
            NextState("WAIT_WRITE_CONFIG_ACK")
        )

        ctrlfsm.act("WAIT_WRITE_CONFIG_ACK",
            If(self.spi.ack_o == 1,
                self.spi.tb_i.eq(0),
                self.p_mem.adr.eq(_index),
                If (_index == len(spi_init),
                    NextState("WRITE_TEST_DATA")
                ). Else(
                    NextState("WRITE_CONFIG_DATA")
                )
            ) # Else wait for ACK
        )

        ctrlfsm.act("WRITE_TEST_DATA",
            self.spi.tb_i.eq(1),
            self.spi.rw_i.eq(1),
            self.spi.dat_i.eq(self.p_mem.dat_r[:8]),    
            self.spi.addr_i.eq(self.p_mem.dat_r[8:]),
            NextValue(_index,_index + 1),
            NextState("WAIT_WRITE_TEST_DATA_ACK")
        )

        ctrlfsm.act("WAIT_WRITE_TEST_DATA_ACK",
            If(self.spi.ack_o == 1,
                self.spi.tb_i.eq(0),  
                If (_index == self.mem.depth,
                    NextState("DONE") #BEN WAS HERE
                ).Else(
                    NextState("READ_STATUS_REGISTER")
                )
            )
        )

        ctrlfsm.act("READ_STATUS_REGISTER",
                self.spi.tb_i.eq(1),
                self.spi.rw_i.eq(0),
                self.spi.addr_i.eq(SPIDEMO.spi_regs["SPITXDR"]),
                NextState("WAIT_FOR_READ_STATUS_REGISTER_ACK")
            )


        ctrlfsm.act("WAIT_FOR_READ_STATUS_REGISTER_ACK",
            If( self.spi.ack_o == 1,
                self.spi.tb_i.eq(0),
                NextState("WAIT_FOR_TX_READY") #BEN WAS HERE
                )
        )

         
        ctrlfsm.act("WAIT_FOR_TX_READY",
            self.spi.tb_i.eq(0),
            self.p_mem.adr.eq(_index),
            If(self.spi.dat_o[4] == 0,
                NextState("READ_STATUS_REGISTER") #BEN WAS HERE
            ).Else(
                NextState("WRITE_TEST_DATA")  
            )
        )

        ctrlfsm.act("DONE",
            self.spi.tb_i.eq(0)
        )


    def tb_spi_demo(self):
        states = list(self.ctrlfsm.actions.keys())
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
        
        yield self.spi.ack_o.eq(1)
        yield self.spi.dat_o.eq(0xff)
        for i in range(100):
            state = yield self.ctrlfsm.state
            index = yield self.index
            addr  = yield self.spi.addr_i
            data  = yield self.spi.dat_i
            spi_strobe = yield self.spi.tb_i
            spi_rw     = yield self.spi.rw_i
            spi_ack_o  = yield self.spi.ack_o
            if (spi_strobe == 1) & (spi_rw == 1):
                print("State: {:<20}:, Index={:<2}, SPI: Addr {:03x}, Data {:02x} ".format(states[state],index,addr,data),end="")
                if addr == SPIDEMO.spi_regs["SPITXDR"]:
                    if (data != ord('\r')):
                        print("[{:c}]".format(data),end="")
                    else:
                        print("[<newline>]",end="")  
                print()
            yield        

    def run_simulation(self):
        run_simulation(self,self.tb_spi_demo(),vcd_name="spi_test.vcd")




if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser(description="Lattice ICE40 SPI Demo Example")
    parser.add_argument("command", nargs=1,choices=["sim","build","configure"])
    args = parser.parse_args()
    command = args.command[0]
    plat = Platform()


    if command == "sim":
        spidemo = SPIDEMO(platform=plat,sim=True)
        print(spidemo.osc.frequency)
        spidemo.run_simulation()
    elif command == "build":
        spidemo = SPIDEMO(platform=plat,sim=False)
        plat.build(spidemo)
    elif command == "configure":
        plat.create_programmer().flash(address=0,bitstream_file="build/top.bin")
    else:
        print("Unknown command")
    




