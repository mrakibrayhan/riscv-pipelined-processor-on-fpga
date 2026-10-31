//====================================================================
// hazard_unit_riscv.v   (RISC-V 5-stage pipelined CPU, Step 7)
//
//   Purely combinational. Implements pipeline_spec.md section 6 and 7:
//     * forwarding selects for the EX stage operands
//     * load-use stall (1 cycle)
//     * control hazard: redirect (branch taken / jal / jalr resolved in EX) and the flushes
//     * halt (ecall / ebreak) and drain request -> stop fetching
//     * stall of the whole front end while the multi-cycle M unit is busy (ex_stall, Step 12;
//       tie to 0 for RV32I only)
//
//   Contract (guaranteed by control_pipe_riscv): reg_write is 0 whenever rd = x0, and bubbles
//   have every control bit 0. So no comparison with x0 is needed here.
//
//   forward_a / forward_b : 00 register file value (includes the WB write-first bypass)
//                           10 EX/MEM alu_result        (priority: newest producer)
//                           01 WB result (ALU or load data)
//
//   Outputs towards the pipeline:
//     hold_front    -> if_stage.hold  (PC, IF/ID, imem output register keep their values)
//     redirect      -> if_stage.redirect, PC <- target, IF/ID.valid <- 0
//     no_fetch      -> if_stage.no_fetch (halted, halting now, or draining for a snapshot)
//     bubble_id_ex  -> id_ex_reg.bubble   (load-use stall, redirect flush, halt)
//     hold_id_ex    -> id_ex_reg.hold     (EX busy with a multi-cycle instruction)
//     bubble_ex_mem -> ex_mem_reg.bubble  (EX busy: no new instruction leaves EX)
//     halt_now      -> sets the sticky 'halted' flag in the top level
//====================================================================
module hazard_unit_riscv (
    // ID stage
    input  [4:0] rs1_d,
    input  [4:0] rs2_d,
    input        uses_rs1_d,        // already gated with IF/ID.valid
    input        uses_rs2_d,

    // EX stage
    input  [4:0] rs1_e,
    input  [4:0] rs2_e,
    input  [4:0] rd_e,
    input        reg_write_e,
    input        mem_read_e,        // load in EX
    input        stop_e,            // stop instruction in EX: ecall / ebreak / branch-or-jal to itself / illegal
    input        branch_e,
    input        jump_e,
    input        taken,             // branch condition from the ALU compare (branch_cond_riscv)
    input        ex_stall,          // EX busy (RV32M, Step 12); 0 for RV32I

    // MEM / WB stages
    input  [4:0] rd_m,
    input        reg_write_m,
    input  [4:0] rd_w,
    input        reg_write_w,

    // system state
    input        stopping,          // sticky: a stop instruction has passed EX (fetch stays off until rst)
    input        drain,             // snapshot request: stop fetching, let the pipe empty

    output reg [1:0] forward_a,
    output reg [1:0] forward_b,
    output       stall_ld,          // load-use stall (also useful for the Stall_Count debug counter)
    output       redirect,
    output       halt_now,
    output       hold_front,
    output       no_fetch,
    output       bubble_id_ex,
    output       hold_id_ex,
    output       bubble_ex_mem
);

    // ---------------- forwarding ----------------
    always @(*) begin
        if      (reg_write_m && (rd_m == rs1_e)) forward_a = 2'b10;
        else if (reg_write_w && (rd_w == rs1_e)) forward_a = 2'b01;
        else                                     forward_a = 2'b00;

        if      (reg_write_m && (rd_m == rs2_e)) forward_b = 2'b10;
        else if (reg_write_w && (rd_w == rs2_e)) forward_b = 2'b01;
        else                                     forward_b = 2'b00;
    end

    // ---------------- load-use stall ----------------
    assign stall_ld = mem_read_e & reg_write_e &
                      ((uses_rs1_d & (rs1_d == rd_e)) | (uses_rs2_d & (rs2_d == rd_e)));

    // ---------------- control hazard / halt ----------------
    // the stop instruction kills everything younger (bubble into EX, no new fetch); a branch / jal to itself
    // is a stop, not a redirect (same as "B ." in the ARM pipeline)
    assign redirect = (jump_e | (branch_e & taken)) & ~stop_e;
    assign halt_now = stop_e;
    assign no_fetch = stopping | halt_now | drain;

    // ---------------- pipeline controls ----------------
    assign hold_front    = stall_ld | ex_stall;
    assign bubble_id_ex  = stall_ld | redirect | halt_now;
    assign hold_id_ex    = ex_stall;
    assign bubble_ex_mem = ex_stall;

endmodule
