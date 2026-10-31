## timing_constraints.sdc  (RISC-V single-cycle CPU)
## Adapted directly from the MIPS single-cycle project's .sdc - same
## structure, same reasoning, retargeted to this project's signal
## names (clk / clkdiv[1] / KEY in top_cpu_riscv.v).
##
## Tells TimeQuest that clk is a real 50MHz clock (period = 20.000 ns).
## Without this, Quartus has no timing constraint on the actual board
## clock domain at all - it only reports Fmax for the unrelated
## internal JTAG test clock (altera_reserved_tck).

create_clock -name clk -period 20.000 [get_ports {clk}]

## cpu_clk is the CLOCK_50/4 divider in top_cpu_riscv.v (clkdiv[1])
## that actually drives the CPU core (cpu_top_riscv). It's a
## generated clock, not a port, so without this line TimeQuest
## silently skips analyzing that domain entirely - it won't even show
## up in the Fmax Summary. This is what lets us see a real Fmax
## number for the domain alu_riscv.v's reused ripple-carry chain
## actually has to meet.
create_generated_clock -name cpu_clk -source [get_ports {clk}] -divide_by 4 [get_registers {clkdiv[1]}]

derive_clock_uncertainty

## KEY[4:0] are plain async push-buttons, not clocked logic - tell
## the analyzer not to time-constrain paths from them.
set_false_path -from [get_ports {KEY[*]}]
