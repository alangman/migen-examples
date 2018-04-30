import sys
sys.path += ["/home/alan/repo/github/alangman/migen-examples/cyclone/lib"]

import max1000
from migen import *
from migen.build.platforms import cyc1000

class Rot(Module):
    def __init__(self,N,clk_freq=12000000):
        self.clk_freq = clk_freq
        self.ready = Signal()
        self.dir = Signal(reset=0)
        self.x = Signal(N)
        self.cnt = Signal(max=N,reset = N-1)
        self.divider = Signal(max=self.clk_freq)
        ###
        self.sync += [
            If(self.ready,
                If(self.divider == int(self.clk_freq//16) - 1,
                    self.divider.eq(0),
                    If (self.dir == 0,
                        self.x.eq(Cat(self.x[-1], self.x[:-1]))
                    ).Else (
                        self.x.eq(Cat(self.x[1:],self.x[0]))
                    ),
                    If (self.cnt == 1,
                        self.dir.eq(~self.dir),
                        self.cnt.eq(N-1)
                    ).Else (
                        self.cnt.eq(self.cnt-1)
                    )
                ).Else(
                    self.divider.eq(self.divider + 1)
                )
            ).Else(
                self.ready.eq(1),
                self.x.eq(1),
                self.divider.eq(0)
            )
        ]
class TopLevel(Module):
    def __init__(self,plat,N=8):
        self.submodules.rot = Rot(N)
        #Connect to io
        self.comb += [ plat.request("user_led").eq(self.rot.x[i]) for i in range(N)]

        

if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser(description="Intel max1000 Flasher Example")
    parser.add_argument("command", nargs=1,choices=["sim","build","configure","verilog"])
    args = parser.parse_args()
    command = args.command[0]
    plat =  cyc1000.Platform()

    if command == "sim":
        pass
    elif command == "build":
        app = TopLevel(plat)
        plat.build(app,bitstream_ext="svf")
    elif command == "configure":
        plat.create_programmer().load_bitstream(bitstream_file="build/top.svf")
    elif command == "verilog":
        app = TopLevel(plat)
        print(verilog.convert(app))
    else:
        print("Unknown command")
    




