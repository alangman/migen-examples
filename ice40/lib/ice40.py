from migen import *

class ICE40ResourceError(Exception):
    pass
class DeviceInterfaceError(Exception):
    pass
class DeviceSimulationError(Exception):
    pass

class OSC(Module):
    Clk = { "48MHz":("0b00","SB_HFOSC",48000000),
            "24MHz":("0b01","SB_HFOSC",24000000),
            "12MHz":("0b10","SB_HFOSC",12000000),
            "6MHz" :("0b11","SB_HFOSC",6000000),
            "10kHz":("0b00","SB_LFOSC",10000)}

    _resource =["SB_HFOSC","SB_LFOSC"]

    def __init__(self,freq = "48MHz",sim=False):
        if not freq in OSC.Clk.keys():
            ICE40ResourceError("<{}> Clk frequency {} not available".format(OSC.__class__,freq))
        if not OSC.Clk[freq][1] in OSC._resource:
            ICE40ResourceError("<{}> Clk resource already used".format(OSC.__class__))
        self._ip = OSC.Clk[freq][1]
        self._freq = OSC.Clk[freq][2]
        OSC._resource.remove(self._ip)

        self.clk = Signal()
        
        if not sim:
            self.specials.osc = Instance(self._ip,
                                        i_CLKHFPU=1,
                                        i_CLKHFEN=1,
                                        o_CLKHF=self.clk)
            if self._ip == "SB_HFOSC":
                self.osc.items.append(Instance.Parameter("CLKHF_DIV",OSC.Clk[freq][0]))                                    
       
    @property
    def frequency(self):
        return self._freq

    def __str__(self):
        return "osc_{}MHz_clk".format(self._freq)


class SB_SPI(Module):
    locations = {"llc":{"BUS_ADDR74":0b0000},
                 "lrc":{"BUS_ADDR74":0b0010},
                "available":["llc","lrc"]
                }
    def __init__(self,pads = None,bus = None,loc=None,sim=False,core_ssn=False):
        #Check to see if resource can be allocatedtb_i.eq(1),
        if not SB_SPI.locations["available"]:
            raise ICE40ResourceError("SB_SPI: all resources allocated")
        if not loc==None:
            if (not loc in SB_SPI.locations) and (not loc=="available"):
                raise ICE40ResourceError("SB_SPI: unregonized location, should be llc,lrc")
            if loc not in SB_SPI.locations["available"]:
                raise ICE40ResourceError("SB_SPI: resouce at location [{}] already allocated".format(loc))
        #Allocate the resource
        self._loc = SB_SPI.locations["available"].pop()
        #define the parameters
       
        #Check external interface
        if pads is None:
             ICE40ResourceError("SB_SPI: No pads defined.")
        self.pads = pads


        #SOC Interface
        self.wkup_o    = Signal()       # SPI Wakeup from Standby signal
        self.irq_o     = Signal()       # SPI Interrupt output   (status register for now, will move to interrupt on SOC)
        self.rw_i      = Signal()       # System Read/Write Input
        self.tb_i      = Signal()       # Strobe Signal
        self.addr_i    = Signal(4)      # System Bus Control address (8 bits)   SBADRI[7:4] must match with BUS_DDR74 for the slice
        self.dat_i     = Signal(8)      # System Data Input (8 bits)
        self.dat_o     = Signal(8)      # System Data Output (8 bits)   
        self.ack_o     = Signal()       # Transaction acknowledge from SPI Core

         
        #Device Interface
        self.mi_i      = Signal()          # Master input from PAD
        self.so_o      = Signal()          # Slave output to PAD
        self.soe_o     = Signal()          # Slave output Enable to PAD  (active high)
        self.si_i      = Signal()          # Slave input from PAD
        self.mo_o      = Signal()          # Master output to PAD
        self.moe_o     = Signal()          # Master output Enable to PAD (active high)
        self.sck_i     = Signal()          # Slave Clock input from PAD
        self.sck_o     = Signal()          # Slave Clock output to PAD
        self.sckoe_o   = Signal()          # Slave Clock output enable to PAD (active high)
        self.scsn_i    = Signal(reset=0b1) # Slave Chip Select input from PAD (select by default)
        self.mcsno_o   = Signal(4,reset_less=True)         # Master Chip Select Output to PAD (4 bits)
        self.mcsnoe_o  = Signal(4)         # Master Chip Select output enable to PAD. (active high)

        #Add bus
        if not bus is None:
            if bus == "wishbone":
                self.add_wishbone_bus()
            else:
                raise DeviceInterfaceError("{} bus is not supported".format(bus))
                
        
        #Internal Signals   (upper bits of SPI is dependant on the core)
        self._bus_addr74 = Signal(4,reset=SB_SPI.locations[self._loc]["BUS_ADDR74"])                       #fixed address
        
        if not sim:
        #Connect SPI output 
            self.specials += Instance("SB_IO", p_PIN_TYPE=0b101001,
                        io_PACKAGE_PIN = self.pads.mosi, o_OUTPUT_ENABLE=self.moe_o, o_D_OUT_0=self.mo_o, o_D_IN_0=self.si_i)
            self.specials += Instance("SB_IO", p_PIN_TYPE=0b101001,
                        io_PACKAGE_PIN = self.pads.miso, o_OUTPUT_ENABLE=self.soe_o, o_D_OUT_0=self.so_o, o_D_IN_0=self.mi_i)
            self.specials += Instance("SB_IO", p_PIN_TYPE=0b101001,p_PULLUP=0b1,
                        io_PACKAGE_PIN = self.pads.sck, o_OUTPUT_ENABLE=self.sckoe_o, o_D_OUT_0=self.sck_o, o_D_IN_0=self.sck_i)
            if core_ssn==True:
                n_ssn = len(pads.ssn)
                for i in range(n_ssn):   #support more than on CS
                    self.comb += If(self.mcsnoe_o,
                                    self.pads.ssn[i].eq(self.mcsno_o[i])
                            )

        #Instantiate Lattice IP
            self.specials += Instance ("SB_SPI",
                                    p_BUS_ADDR74  = "0b{:04b}".format(SB_SPI.locations[self._loc]["BUS_ADDR74"]),             # '0b0001'
                                    i_SBCLKI      = ClockSignal(),
                                    i_SBRWI       = self.rw_i,
                                    i_SBSTBI      = self.tb_i,
                                    i_SBADRI0     = self.addr_i[0],
                                    i_SBADRI1     = self.addr_i[1],
                                    i_SBADRI2     = self.addr_i[2],
                                    i_SBADRI3     = self.addr_i[3],
                                    i_SBADRI4     = self._bus_addr74[0],
                                    i_SBADRI5     = self._bus_addr74[1],
                                    i_SBADRI6     = self._bus_addr74[2],
                                    i_SBADRI7     = self._bus_addr74[3],
                                    i_SBDATI0     = self.dat_i[0],
                                    i_SBDATI1     = self.dat_i[1],
                                    i_SBDATI2     = self.dat_i[2],
                                    i_SBDATI3     = self.dat_i[3],
                                    i_SBDATI4     = self.dat_i[4],
                                    i_SBDATI5     = self.dat_i[5],
                                    i_SBDATI6     = self.dat_i[6],
                                    i_SBDATI7     = self.dat_i[7],
                                    o_SBDATO0     = self.dat_o[0],
                                    o_SBDATO1     = self.dat_o[1],
                                    o_SBDATO2     = self.dat_o[2],
                                    o_SBDATO3     = self.dat_o[3],
                                    o_SBDATO4     = self.dat_o[4],
                                    o_SBDATO5     = self.dat_o[5],
                                    o_SBDATO6     = self.dat_o[6],
                                    o_SBDATO7     = self.dat_o[7],
                                    i_MI          = self.mi_i,
                                    o_SO          = self.so_o,
                                    o_SOE         = self.soe_o,
                                    i_SI          = self.si_i,
                                    o_MO          = self.mo_o,
                                    o_MOE         = self.moe_o,
                                    i_SCKI        = self.sck_i,
                                    o_SCKO        = self.sck_o,
                                    o_SCKOE       = self.sckoe_o,
                                    i_SCSNI       = self.scsn_i,
                                    o_SBACKO      = self.ack_o,          
                                    o_SPIIRQ      = self.irq_o,      
                                    o_SPIWKUP     = self.wkup_o,
                                    o_MCSNO3      = self.mcsno_o[3],
                                    o_MCSNO2      = self.mcsno_o[2],
                                    o_MCSNO1      = self.mcsno_o[1],
                                    o_MCSNO0      = self.mcsno_o[0],
                                    o_MCSNOE3     = self.mcsnoe_o[3],
                                    o_MCSNOE2     = self.mcsnoe_o[2],
                                    o_MCSNOE1     = self.mcsnoe_o[1],
                                    o_MCSNOE0     = self.mcsnoe_o[0]
            )

        def add_wishbone_bus(self,data_width=8):
            self. bus = wishbone.Interface(data_width=data_width)
            self.sync += [
                            self.rw_i.eq(self.wishbone.we),
                            self.tb_i.eq(self.wishbone.cyc & self.wishbone.stb),
                            self.addr_i.eq(self.wishbone.adr[:4]),     #upper four bits fixed for chosen core
                            self.dat_i.eq(self.wishbone.dat_w),
                            self.wishbone.dat_r.eq(self.dat_o)
                        ]
            self.submodules.fsm = FSM(reset_state="IDLE")
            self.fsm.act("IDLE",
                NextValue(self.wishbone.ack,0), 
                NextValue(self.tb_i,0),
                If  (self.wishbone.cyc & self.wishbone.stb,
                        NextValue(self.tb_i,1),
                        NextState("WAIT_SPI_ACK")
                    )
                )
            self.fsm.act("WAIT_SPI_ACK",              #Should add a check for a bus error!
                If (self.ack_o == 1,
                    NextValue(self.tb_i,0),   
                    NextState("SEND_WB_ACK")
                    )
                )
            self.fsm.act("SEND_WB_ACK",
                NextValue(self.wishbone.ack,1),
                NextState("IDLE")
                )

        def spi_sim_model(self,delay=2):
            if (delay > 255):
                raise DeviceSimulationError("Maximum SPI ack delay must be less than 256 clock cycles")
            ack_delay = Signal(8,reset=delay)
            self.submodules.spi_model = FSM(reset_state="IDLE")
            self.fsm.act("IDLE",
                NextValue(self.ack_o,0), 
                    If  (self.tb_i == 1,
                        NextState("DONE")
                    )
            )
            self.fsm.act("DONE",
                If (self.tb_i == 0,
                    NextValue(ack_delay,delay),
                    NextStat("WAIT")
                    )
            )
            self.fsm.act("WAIT",
                If (ack_delay == 0,
                    NextValue(self.ack_o,1),
                    NextState("IDLE")
                ),
                NextValue(ack_delay,ack_delay-1)
            )



