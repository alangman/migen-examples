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
    
        #Configure oscillator and clock
        self.submodules.osc = OSC(freq="12MHz",sim=sim)
        if not sim:
            self.submodules.crg = CRG(clk = self.osc.clk)
          
        #Configure simple led blinker
        self.submodules.blinker = Blinker(platform.request("led"),clk_freq=self.osc.frequency,period=0.5)
        #Configure lattice spi
        self.submodules.spi = SB_SPI(pads = platform.request("spi"), sim=sim)
        #Enable SPI core
        self.comb += self.spi.scsn_i.eq(1)
     
        
        #Add WaitTimer module
        self.submodules.delay = WaitTimer(100)     #100 clock cycle delay

        #Define simple state machines to write init and test string to SB_SPI
        self.submodules.ctrlfsm  = ctrlfsm = FSM()

        self.data_counter = Signal(8,reset=0)
        self.delay_counter = Signal(8,reset=10)

        #Testing
        self.test = platform.request("test") 
        self.comb += [ self.test.p1.eq(ClockSignal())
        ]
        

        ctrlfsm.act("BEGIN",
                self.spi.rw_i.eq(0),
                self.spi.tb_i.eq(0),
                If(self.delay_counter==0,
                    NextState("STATE0")
                    ),
                NextValue(self.test.p2,0),
                NextValue(self.delay_counter,self.delay_counter-1)
        )
        ctrlfsm.act("STATE0",
                self.spi.addr_i.eq(SPIDEMO.spi_regs["SPICR1"]),
                self.spi.dat_i.eq(spi_init[0][1]),
                self.spi.rw_i.eq(1),
                self.spi.tb_i.eq(1),
                NextState("STATE1")
        )

        ctrlfsm.act("STATE1",
                If(self.spi.ack_o == 1,
                    self.spi.rw_i.eq(0),
                    self.spi.tb_i.eq(0),
                    NextState("STATE2")
                ),
                 NextValue(self.test.p2,~self.test.p2),
        )

        ctrlfsm.act("STATE2",       #Set divider
                self.spi.addr_i.eq(SPIDEMO.spi_regs["SPIBR"]),
                self.spi.dat_i.eq(spi_init[1][1]),
                self.spi.rw_i.eq(1),
                self.spi.tb_i.eq(1),
                NextState("STATE3")
        )

        ctrlfsm.act("STATE3",      
                If(self.spi.ack_o == 1,
                    self.spi.rw_i.eq(0),
                    self.spi.tb_i.eq(0),
                    NextState("STATE4")
                )
        )

        ctrlfsm.act("STATE4",       #Set master mode
                self.spi.addr_i.eq(SPIDEMO.spi_regs["SPICR2"]),
                self.spi.dat_i.eq(spi_init[2][1]),
                self.spi.rw_i.eq(1),
                self.spi.tb_i.eq(1),
                NextState("STATE5")
        )

        ctrlfsm.act("STATE5",      
                If(self.spi.ack_o == 1,
                    self.spi.rw_i.eq(0),
                    self.spi.tb_i.eq(0),
                    NextState("STATE6")
                )
        )


        ctrlfsm.act("STATE6",       #SEND BYTES
                self.spi.addr_i.eq(SPIDEMO.spi_regs["SPITXDR"]),
                self.spi.dat_i.eq(self.data_counter),
                self.spi.rw_i.eq(1),
                self.spi.tb_i.eq(1),
                NextState("STATE7")
        )

        ctrlfsm.act("STATE7",     
                If(self.spi.ack_o == 1,
                    self.spi.rw_i.eq(0),
                    self.spi.tb_i.eq(0),
                    NextValue(self.data_counter,self.data_counter + 1),
                    NextState("STATE8")
                )
        )

        ctrlfsm.act("STATE8",       #READ Status
                self.spi.addr_i.eq(SPIDEMO.spi_regs["SPISR"]),
                self.spi.rw_i.eq(0),
                self.spi.tb_i.eq(1),
                NextState("STATE9")
        )

        ctrlfsm.act("STATE9",      
                If(self.spi.ack_o == 1,
                    self.spi.rw_i.eq(0),
                    self.spi.tb_i.eq(0),
                    NextState("STATE10")
                )
        )

        ctrlfsm.act("STATE10",      
                NextValue(self.delay_counter,10),
                If(self.spi.dat_o[4] == 1,
                    If (self.data_counter == 10,
                        NextState("STATE11"),
                    ).Else(
                        NextState("STATE6")
                    )
                ).Else(
                    NextState("STATE8")
                )
        )

        ctrlfsm.act("STATE11",
                If(self.delay_counter == 0,
                    NextState("NOP")
                ),
                NextValue(self.delay_counter,self.delay_counter-1)
        )

        ctrlfsm.act("NOP",
                NextValue(self.data_counter,0),
                NextState("STATE6")
        )
  

    def tb_spi_demo(self):
        print("[TEST 1] Test SPI Configuration and DATA transmit")
        yield from self.tb_spi_controller()
        print("-------------------------")

         
    def tb_spi_controller(self):
        states = list(self.ctrlfsm.actions.keys())      #names of states
        yield self.spi.ack_o.eq(1)
        yield self.spi.dat_o.eq(0xff)
        bus_address = yield self.spi._bus_addr74
        print(bus_address)
        _run=True
        _cycles = 0
        while _run:
            state = yield self.ctrlfsm.state
            data_counter = yield self.data_counter
            delay_counter = yield self.delay_counter
            addr  = yield self.spi.addr_i
            data  = yield self.spi.dat_i
            spi_strobe  = yield self.spi.tb_i
            spi_rw      = yield self.spi.rw_i
            spi_ack_o   = yield self.spi.ack_o
            bus_address = yield self.spi._bus_addr74
            if (states[state]=="STATE6"):
                print("State: {:<20}: Data Counter={:<4}, SPI: Addr {:02x}, Data {:02x}, Cycles {:<5} Bus Address {:02x}".format(states[state],data_counter,addr,data,_cycles,bus_address))      
            if states[state] == "NOP":
                _run=False
            _cycles += 1 
            yield


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
    




