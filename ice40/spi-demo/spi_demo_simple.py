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
    
    SPICR0  =0x08
    SPICR1  =0x09
    SPICR2  =0x0A
    SPIBR   =0x0B
    SPITXDR =0x0D
    SPIRXDR =0x0E
    SPICSR  =0x0F
    SPISR   =0x0C
    SPIINTSR=0x06
    SPIINTCR=0x07


    def __init__(self,platform=None,pads_led = None,sim=False):
        #Configure oscillator and clock
        self.submodules.osc = OSC(freq="12MHz",sim=sim)
        if not sim:
            self.submodules.crg = CRG(clk = self.osc.clk)
          
        #Configure simple led blinker
        self.submodules.blinker = Blinker(platform.request("led"),clk_freq=self.osc.frequency,period=0.5)
        #Configure lattice spi
        pads_spi = platform.request("spi")
        self.submodules.spi = SB_SPI(pads = pads_spi, sim=sim)
        #Enable SPI core
        self.comb += self.spi.scsn_i.eq(1)
     

        #Define simple state machines to write init and test string to SB_SPI
        self.submodules.ctrlfsm  = ctrlfsm = FSM()

        self.data_counter = Signal(8,reset=0)
        self.delay_counter = Signal(32,reset=10)

        #Testing signals
        self.test = platform.request("test") 
        self.comb += [self.test.p0.eq(ClockSignal())
        ]

        self.sync += [  self.test.p1.eq(self.spi.ack_o),
    
        ]

        spi_busy=Signal()
        spi_tx_ready=Signal()
        self.spi_tx_ready=spi_tx_ready

        self.comb+= [
            spi_busy.eq(self.spi.dat_o[7]),
            spi_tx_ready.eq(self.spi.dat_o[4])
        ]
        
        if sim:
            delay_max_bit = 1
        else:
            delay_max_bit = 10

        ctrlfsm.act("INIT0",
                    NextValue(self.delay_counter,10),
                    NextValue(self.spi.rw_i,0),
                    NextValue(self.spi.tb_i,0),
                    NextValue(self.test.p2,0),
                    NextValue(pads_spi.ssn,1),
                    NextState("INIT1")
        )

        ctrlfsm.act("INIT1",
                    If(self.delay_counter == 0,
                        NextState("STATE0")
                    ).Else(
                        NextValue(self.delay_counter,self.delay_counter-1)
                    )
        )
        
        ctrlfsm.act("STATE0",
                NextValue(self.spi.addr_i,SPIDEMO.SPICR1),
                NextValue(self.spi.dat_i,0x80),
                NextValue(self.spi.rw_i,1),
                NextValue(self.spi.tb_i,1),
                NextState("STATE1")
        )


        ctrlfsm.act("STATE1",
                If(self.spi.ack_o == 0,
                    NextState("STATE1")
                ).Else(
                    NextValue(self.spi.rw_i,0),
                    NextValue(self.spi.tb_i,0),
                    NextState("STATE2")
                )
        )

        ctrlfsm.act("STATE2",       #Set divider
                NextValue(self.spi.addr_i,SPIDEMO.SPIBR),
                NextValue(self.spi.dat_i,0x3f),
                NextValue(self.spi.rw_i,1),
                NextValue(self.spi.tb_i,1),
                NextState("STATE3")
        )

        ctrlfsm.act("STATE3",      
                If(self.spi.ack_o == 0,
                    NextState("STATE3")
                ).Else(
                    NextValue(self.spi.rw_i,0),
                    NextValue(self.spi.tb_i,0),
                    NextState("STATE4")
                )
        )

        ctrlfsm.act("STATE4",       #Set master mode
                NextValue(self.spi.addr_i,SPIDEMO.SPICR2),
                NextValue(self.spi.dat_i,0xc4),
                NextValue(self.spi.rw_i,1),
                NextValue(self.spi.tb_i,1),
                NextState("STATE5")
        )

        ctrlfsm.act("STATE5",      
                If(self.spi.ack_o == 0,
                    NextState("STATE5")
                ).Else(
                    NextValue(self.spi.rw_i,0),
                    NextValue(self.spi.tb_i,0),
                    NextState("STATE6")
                )
        )


        ctrlfsm.act("STATE6",       #SEND BYTES
                NextValue(pads_spi.ssn,1),
                NextValue(self.spi.dat_i,self.data_counter),
                NextValue(self.data_counter,self.data_counter+1),
                NextValue(self.spi.addr_i,SPIDEMO.SPITXDR),
                NextValue(self.spi.rw_i,1),
                NextValue(self.spi.tb_i,1),
                NextState("STATE7")
        )

        ctrlfsm.act("STATE7",    
                If(self.spi.ack_o == 0,
                    NextState("STATE7")
                ).Else(
                    NextValue(self.spi.rw_i,0),
                    NextValue(self.spi.tb_i,0),
                    NextState("STATE8")
                )
        )

        ctrlfsm.act("STATE8",       #READ Status
                NextValue(self.spi.rw_i,0),
                NextValue(self.spi.tb_i,1),
                NextValue(self.spi.addr_i,SPIDEMO.SPISR),
                NextState("STATE9")
        )

        ctrlfsm.act("STATE9",      
                If(self.spi.ack_o == 0,
                    NextState("STATE9")

                ).Else(
                    NextValue(self.spi.rw_i,0),
                    NextValue(self.spi.tb_i,0),
                    NextState("STATE10")
                )
        )

        ctrlfsm.act("STATE10",      
                NextValue(self.delay_counter,0),
                If(self.spi.dat_o[4] == 0,
                        NextState("STATE8"),    #Could also just wait around here.
                        NextValue(self.test.p2,1)
                    ).Else (
                        NextValue(pads_spi.ssn,0),
                        NextState("STATE11")
                    )
        )

        ctrlfsm.act("STATE11",
                NextValue(self.test.p2,0),
                If(self.delay_counter[delay_max_bit]==1,
                    NextState("STATE6")
                ).Else(
                    NextValue(self.delay_counter,self.delay_counter+1),
                    NextState("STATE11")
                )
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
        spi_tx_ready = yield self.spi_tx_ready
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
            if (states[state]=="STATE7"):
                print("State: {:<20}: Data Counter={:<4}, SPI: Addr {:02x}, Data {:02x}, Cycles {:<5} Bus Address {:02x} TX Ready: {}".format(states[state],data_counter,addr,data,_cycles,bus_address,spi_tx_ready))      
            if states[state] == "STATE11" and data_counter == 10:
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
    




