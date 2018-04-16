from migen import *
from migen.genlib.io import CRG
from migen.genlib.misc import WaitTimer
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
                    ("SPIBR", 0x23),     #Set Divider to 23  for 1MHz SPI CLK  (div = DIVIDER[5..0]+1)
                    ("SPICSR",0x01),     #CSN_0 as chip select
                    ("SPICR2",0xC0)      #Set Master Mode CPOL=0,CPHA=0,LDBF=0
                    ]
        
        spi_test_str = b'Hello World, And Welcome\r'

        #Create and initialize spi init sequence memory    addr[4..0]data[7..0]
        mem_depth = len(spi_init) + len(spi_test_str) + 1   #include terminate value
        mem_width = 12   #4 bit Addr, 8 bit Data
        mem_i_config   =  [(SPIDEMO.spi_regs[e[0]] <<8 | e[1])       for e in spi_init]
        mem_i_test_str =  [(SPIDEMO.spi_regs["SPITXDR"] << 8 | char) for char in spi_test_str]
        self.specials.mem = Memory(width=mem_width,depth=mem_depth,init=mem_i_config + mem_i_test_str)
        self.specials.p_mem = self.mem.get_port(write_capable=False)
        #Configure oscillator and clock
        self.submodules.osc = OSC(freq="12MHz",sim=sim)
        if not sim:
            self.submodules.crg = CRG(clk = self.osc.clk)
          
        #Configure simple led blinker
        self.submodules.blinker = Blinker(platform.request("led"),clk_freq=self.osc.frequency,period=0.5)
        #Configure lattice spi
        self.submodules.spi = SB_SPI(pads = platform.request("spi"), sim=sim)

        #Index to controller memory
        self.index = _index = Signal(8,reset=0)   #Make sure the width of index is the same as the addr of the mem
        
        #Add WaitTimer module
        self.submodules.delay = WaitTimer(100)     #100 clock cycle delay

        #Define simple state machines to write init and test string to SB_SPI
        self.submodules.ctrlfsm  = ctrlfsm = FSM()

        self.comb += [
                    self.p_mem.adr.eq(_index),
                    self.spi.scsn_i.eq(1),
        ]
        

        #Testing
        self.test = platform.request("test") 
        self.comb += [ self.test.p1.eq(ClockSignal())
        ]
        
        ctrlfsm.act("INIT",
            self.test.p0.eq(1),
            NextValue(self.test.p2,1),
            self.delay.wait.eq(0),           #Counter disable
            NextValue(_index,0),
            #add delay other setup.
            NextState("WRITE_CONFIG_DATA")
        )


        ctrlfsm.act("WRITE_CONFIG_DATA",
            self.spi.tb_i.eq(1),
            self.spi.rw_i.eq(1),
            self.spi.dat_i.eq(self.p_mem.dat_r[:8]),    
            self.spi.addr_i.eq(self.p_mem.dat_r[8:]),
            NextState("WAIT_WRITE_CONFIG_ACK")
        )

        ctrlfsm.act("WAIT_WRITE_CONFIG_ACK",
            If(self.spi.ack_o == 1,
                self.spi.tb_i.eq(0),
                NextValue(self.test.p2,0),
                NextValue(_index,_index + 1),
                If (_index == len(spi_init),
                    NextState("WRITE_TEST_DATA")
                ). Else(
                    NextState("WRITE_CONFIG_DATA")
                ),
            ),
            NextValue(self.test.p2,~self.test.p2),
            self.spi.dat_i.eq(self.p_mem.dat_r[:8]),    
            self.spi.addr_i.eq(self.p_mem.dat_r[8:]),
             # Else wait for ACK
        )

        ctrlfsm.act("WRITE_TEST_DATA",
            self.spi.tb_i.eq(1),
            self.spi.rw_i.eq(1),
            self.spi.dat_i.eq(self.p_mem.dat_r[:8]),    
            self.spi.addr_i.eq(self.p_mem.dat_r[8:]),
            NextState("WAIT_WRITE_TEST_DATA_ACK"),
            NextValue(self.test.p2,0)
        )

        ctrlfsm.act("WAIT_WRITE_TEST_DATA_ACK",
            If(self.spi.ack_o == 1,
                self.spi.tb_i.eq(0),  
                 NextValue(_index,_index + 1),
                If (_index == self.mem.depth,
                    NextState("END_DELAY") 
                ).Else(
                    NextState("READ_STATUS_REGISTER")
                )
            ),
            self.spi.dat_i.eq(self.p_mem.dat_r[:8]),    
            self.spi.addr_i.eq(self.p_mem.dat_r[8:]),
        )

        ctrlfsm.act("READ_STATUS_REGISTER",
                self.spi.tb_i.eq(1),
                self.spi.rw_i.eq(0),
                NextValue(self.spi.addr_i,SPIDEMO.spi_regs["SPISR"]),
                NextState("WAIT_FOR_READ_STATUS_REGISTER_ACK")
            )


        ctrlfsm.act("WAIT_FOR_READ_STATUS_REGISTER_ACK",
            If( self.spi.ack_o == 1,
                self.spi.tb_i.eq(0),
                NextState("WAIT_FOR_TX_READY") 
                )
        )

         
        ctrlfsm.act("WAIT_FOR_TX_READY",
                self.spi.tb_i.eq(0),
                If(self.spi.dat_o[4] == 0,
                    NextState("READ_STATUS_REGISTER") 
                ).Else(
                    NextState("WRITE_TEST_DATA")  
                )
        )

        ctrlfsm.act("END_DELAY",
            self.test.p0.eq(0),
            self.spi.tb_i.eq(0),
            self.delay.wait.eq(1),             #Start timer    
            NextState("END")
        )
        
        ctrlfsm.act("END",
                    self.delay.wait.eq(1),
                    If(self.delay.done,
                        NextState("INIT")
                    )
        )
    


      

    def tb_spi_demo(self):
        print("[TEST 1] Read init memory")
        yield from self.tb_command_memory()
        print("-------------------------")
        print("[TEST 2] Test SPI Configuration and DATA transmit")
        yield from self.tb_spi_controller()
        print("-------------------------")

         
    def tb_spi_controller(self):
        states = list(self.ctrlfsm.actions.keys())      #names of states
        yield self.spi.ack_o.eq(1)
        yield self.spi.dat_o.eq(0xff)
        _run=True
        _cycles = 0
        while _run:
            state = yield self.ctrlfsm.state
            index = yield self.index
            addr  = yield self.spi.addr_i
            data  = yield self.spi.dat_i
            spi_strobe = yield self.spi.tb_i
            spi_rw     = yield self.spi.rw_i
            spi_ack_o  = yield self.spi.ack_o
            delay_wait = yield self.delay.wait
            delay_done = yield self.delay.done
            if (spi_strobe == 1) & (spi_rw == 1):
                print("State: {:<20}: Index={:<2}, SPI: Addr {:03x}, Data {:02x} ".format(states[state],index,addr,data),end="")
                if addr == SPIDEMO.spi_regs["SPITXDR"]:
                    if (data != ord('\r')):
                        print("[{:c}]".format(data),end="")
                    else:
                        print("[<newline>]",end="")  
                print()
            if states[state] == "END_DELAY":
                print("State: {:<20}:  Cycle count = {:04d}, Done={}, Wait={}".format(states[state],_cycles,delay_done,delay_wait))
            if states[state] == "END" and delay_done == 1:
                print("State: {:<20}:  Cycle count = {:04d}, Done={}, Wait={}".format(states[state],_cycles,delay_done,delay_wait))
                _run = False
            _cycles += 1
            yield        

    def tb_command_memory(self):
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


    def run_simulation(self):
        run_simulation(self,self.tb_spi_demo(),vcd_name="spi_test.vcd")




if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser(description="Lattice ICE40 SPI Demo Example")
    parser.add_argument("command", nargs=1,choices=["sim","build","configure","verilog"])
    args = parser.parse_args()
    command = args.command[0]
    plat = Platform()


    if command == "sim":
        spidemo = SPIDEMO(platform=plat,sim=True)
        spidemo.run_simulation()
    elif command == "build":
        spidemo = SPIDEMO(platform=plat,sim=False)
        plat.build(spidemo)
    elif command == "configure":
        plat.create_programmer().flash(address=0,bitstream_file="build/top.bin")
    elif command == "verilog":
        spidemo = SPIDEMO(platform=plat,sim=False)
        print(verilog.convert(spidemo))
    else:
        print("Unknown command")
    




