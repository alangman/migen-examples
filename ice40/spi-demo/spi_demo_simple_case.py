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
    
    def __init__(self,platform=None,sim=False,BUS_ADDR74 = 0b0010):

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
        

        self._bus_addr74 = _bus_addr74 = Signal(4,reset=BUS_ADDR74)
        

        #SOC Interface
        self.wkup         = wkup = Signal()       # SPI Wakeup from Standby signal
        self.irq          = irq  = Signal()       # SPI Interrupt output   (status register for now, will move to interrupt on SOC)

         
        #Device Interface
        mi      = Signal()          # Master input from PAD
        so      = Signal()          # Slave output to PAD
        soe     = Signal()          # Slave output Enable to PAD  (active high)
        si      = Signal()          # Slave input from PAD
        mo      = Signal()          # Master output to PAD
        moe     = Signal()          # Master output Enable to PAD (active high)
        scki    = Signal()          # Slave Clock input from PAD
        scko    = Signal()          # Slave Clock output to PAD
        sckoe   = Signal()          # Slave Clock output enable to PAD (active high)

        mcsno   = Signal(4)         # Master Chip Select Output to PAD (4 bits)
        mcsnoe  = Signal(4)         # Master Chip Select output enable to PAD. (active high)
        
      

        wstrb          = Signal()              # System Read/Write Input
        strobe         = Signal(reset=0)       # Strobe Signal
        address        = Signal(4)             # System Bus Control address (8 bits)   SBADRI[7:4] must match with BUS_DDR74 for the slice
        write_data     = Signal(8)             # System Data Input (8 bits)
        spi_read_data  = Signal(8)             # System Data Output (8 bits)   
        spi_ack        = Signal()              # Transaction acknowledge from SPI Core

        cnt           = Signal(10,reset=0)
        data_counter  = Signal(8,reset=0)
        state         = Signal(4,reset=0)
        
        if sim:
            delay_bit = 1
        else:
            delay_bit = 9

        wait_ack = lambda val: [If(spi_ack==val,
                                    state.eq(state)
                                ).Else(
                                    wstrb.eq(0),strobe.eq(0),state.eq(state+1)
                                )]
        write = lambda reg,dat : [write_data.eq(dat),address.eq(reg),wstrb.eq(1),strobe.eq(1),state.eq(state+1)]

        read  = lambda reg     : [address.eq(reg),wstrb.eq(0),strobe.eq(1),state.eq(state+1)]

        wait_status = lambda bit,t_state,f_state : [If(spi_read_data[bit]==0,
                                                            state.eq(t_state),
                                                    ).Else(
                                                            state.eq(f_state)
                                                    )]
        delay = lambda bit,c_state,n_state : [If(cnt[bit] == 1,
                                                        state.eq(n_state),
                                                ).Else(
                                                        state.eq(c_state),
                                                        cnt.eq(cnt+1)
                                                )]

        self.sync += \
                Case(state, {
                    0b0000:         write(SPICR1,0x80),
                    0b0001:         wait_ack(0),
                    0b0010:         write(SPIBR,0x3f),
                    0b0011:         wait_ack(0),
                    0b0100:         write(SPICR2,0xC0),
                    0b0101:         wait_ack(0),
                    0b0110:         write(SPITXDR,data_counter)+ [data_counter.eq(data_counter+1)],
                    0b0111:         wait_ack(0) + [data_counter.eq(data_counter)],
                    0b1000:         read(SPISR),
                    0b1001:         wait_ack(0),
                    0b1010:         wait_status(4,0b1000,0b1100),
                    0b1100:         delay(delay_bit,0b1100,0b0110),
                    "default":      state.eq(state),
                }) 




        if not sim:
            clk =Signal()
            self.specials += Instance(  "SB_HFOSC",
                                        "CLKHF_DIV","0b10",
                                        i_CLKHFPU=1,
                                        i_CLKHFEN=1,
                                        o_CLKHF=clk
                                    )
            self.submodules += CRG(clk)

            pads_spi = platform.request("spi")
            test = platform.request("test")
            self.comb += test.p0.eq(clk)

        #Configure simple led blinker
            self.submodules.blinker = Blinker(platform.request("led"),12e6,period=0.5)

            self.specials += Instance("SB_IO", p_PIN_TYPE=0b101001,
                        io_PACKAGE_PIN = pads_spi.mosi, o_OUTPUT_ENABLE=moe, o_D_OUT_0=mo, o_D_IN_0=si)
            self.specials += Instance("SB_IO", p_PIN_TYPE=0b101001,
                        io_PACKAGE_PIN = pads_spi.miso, o_OUTPUT_ENABLE=soe, o_D_OUT_0=so, o_D_IN_0=mi)
            self.specials += Instance("SB_IO", p_PIN_TYPE=0b101001,
                        io_PACKAGE_PIN = pads_spi.sck, o_OUTPUT_ENABLE=sckoe, o_D_OUT_0=scko, o_D_IN_0=scki)

            #n_ssn = len(pads_spi.ssn)
            #for i in range(n_ssn):   #support more than on CS
            #        self.comb += If(mcsnoe[i],pads_spi.ssn[i].eq(mcsno[i]))

            self.specials += Instance ("SB_SPI",
                                    p_BUS_ADDR74  = "0b{:04b}".format(BUS_ADDR74),
                                    i_SBCLKI      = clk,
                                    i_SBRWI       = wstrb,
                                    i_SBSTBI      = strobe,
                                    i_SBADRI0     = address[0],
                                    i_SBADRI1     = address[1],
                                    i_SBADRI2     = address[2],
                                    i_SBADRI3     = address[3],
                                    i_SBADRI4     = _bus_addr74[0],
                                    i_SBADRI5     = _bus_addr74[1],
                                    i_SBADRI6     = _bus_addr74[2],
                                    i_SBADRI7     = _bus_addr74[3],
                                    i_SBDATI0     = write_data[0],
                                    i_SBDATI1     = write_data[1],
                                    i_SBDATI2     = write_data[2],
                                    i_SBDATI3     = write_data[3],
                                    i_SBDATI4     = write_data[4],
                                    i_SBDATI5     = write_data[5],
                                    i_SBDATI6     = write_data[6],
                                    i_SBDATI7     = write_data[7],
                                    o_SBDATO0     = spi_read_data[0],
                                    o_SBDATO1     = spi_read_data[1],
                                    o_SBDATO2     = spi_read_data[2],
                                    o_SBDATO3     = spi_read_data[3],
                                    o_SBDATO4     = spi_read_data[4],
                                    o_SBDATO5     = spi_read_data[5],
                                    o_SBDATO6     = spi_read_data[6],
                                    o_SBDATO7     = spi_read_data[7],
                                    i_MI          = mi,
                                    o_SO          = so,
                                    o_SOE         = soe,
                                    i_SI          = si,
                                    o_MO          = mo,
                                    o_MOE         = moe,
                                    i_SCKI        = scki,
                                    o_SCKO        = scko,
                                    o_SCKOE       = sckoe,
                                    i_SCSNI       = 1,
                                    o_SBACKO      = spi_ack,          
                                    o_SPIIRQ      = irq,      
                                    o_SPIWKUP     = wkup,
                                    o_MCSNO3      = mcsno[3],
                                    o_MCSNO2      = mcsno[2],
                                    o_MCSNO1      = mcsno[1],
                                    o_MCSNO0      = mcsno[0],
                                    o_MCSNOE3     = mcsnoe[3],
                                    o_MCSNOE2     = mcsnoe[2],
                                    o_MCSNOE1     = mcsnoe[1],
                                    o_MCSNOE0     = mcsnoe[0]
            )



    #for simulation
        self.wstrb         = wstrb
        self.strobe        = strobe
        self.address       = address
        self.write_data    = write_data
        self.spi_read_data = spi_read_data
        self.spi_ack       = spi_ack

        self.cnt            = cnt
        self.data_counter   = data_counter
        self.state          = state


    def tb_spi_demo(self):
        print("[TEST 1] Test SPI Configuration and DATA transmit")
        yield from self.tb_spi_controller()
        print("-------------------------")

         
    def tb_spi_controller(self):
        states = []
        yield self.spi_ack.eq(1)
        yield self.spi_read_data.eq(0xFF)
        bus_address = yield self._bus_addr74
        print(bus_address)
        _run=True
        _cycles = 0
        while _run:
            state           = yield self.state
            data_counter    = yield self.data_counter
            cnt             = yield self.cnt
            addr            = yield self.address
            data            = yield self.write_data
            spi_strobe      = yield self.strobe
            spi_rw          = yield self.wstrb
            spi_ack         = yield self.spi_ack
            #if (states[state]=="STATE7"):
            print("State: {:04d}: Data Counter={:<4}, SPI: Addr {:02x}, Data {:02x}, Cycles {:<5} Bus Address {}".format(state,data_counter,addr,data,_cycles,bus_address))      
            #if states[state] == "STATE11" and data_counter == 10:
            #        _run=False
            _cycles += 1 
            yield
            if (_cycles == 100):
                _run=False



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
    




