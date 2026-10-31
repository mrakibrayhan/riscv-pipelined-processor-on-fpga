//====================================================================
// top_cpu_riscv.v  (board top-level - RZ-EasyFPGA A2.2 / Cyclone IV EP4CE6)
//
//   RISC-V RV32IM 5-STAGE PIPELINED CPU (synchronous-read M9K memories), same debug structure as the
//   multi-cycle project:
//
//     * Register view and data-memory view: snapshot RAMs R00 / R16 / D00 / D16 (16 x 32 each) read with
//       Tools -> In-System Memory Content Editor. When the program halts (ECALL / EBREAK), or when the
//       ISSP source bit is set, the pipeline is DRAINED (fetch stops, older instructions finish), the CPU
//       is frozen and x0..x31 / data words 0..31 are copied into the RAMs (snapshot_unit.v).
//       The program stops like the ARM pipeline: ECALL / EBREAK / a branch or jal to itself (the
//       "done: beq x2,x2,done" line of the Harris & Harris test riscvtest.s) -> halted; an illegal
//       instruction -> fault. Both take effect when the stop instruction reaches WB.
//       Clear the source bit to let the CPU continue (no instruction is lost or repeated).
//     * ONE ISSP instance: the SAME cpu_issp_status IP as the multi-cycle project (196 bits probe,
//       1 bit source = manual snapshot request). NEW probe layout:
//           [0] started  [1] halted  [2] last branch taken  [3] fault (illegal instruction stopped the CPU)
//           [35:4]    instruction in ID (valid bit is not shown)
//           [51:36]   PC of that instruction (low 16 bits); after a halt / fault: the STOP PC
//                     (branch / jal to itself -> its own address, ECALL / EBREAK / illegal -> address + 4)
//           [67:52]   M-STALL COUNTER    (cycles EX was held by mul / div / rem; 16 bits, saturates at 65535)
//           [99:68]   CYCLE COUNTER      (enabled cpu_clk cycles from run until the halt instruction's WB)
//           [131:100] INSTRUCTION COUNTER (retired instructions, halt instruction included)
//           [163:132] LOAD-USE STALL COUNTER
//           [195:164] REDIRECT COUNTER   (taken branches + jal + jalr = 2 flushed cycles each)
//       CPI = cycle counter / instruction counter.
//       Check: cycles = instructions + 4 + load-use stalls + 2 * redirects + M stalls
//              (a mul / mulh* costs 34 M-stall cycles, div / divu / rem / remu 38).
//
//   Pins, buttons, clock divider and debounce are unchanged:
//     KEY[3] = reset (hold it: the register-file RAM needs >= 32 cpu_clk cycles to clear),
//     KEY[2] = run.  LEDG (active low = lit): [0] fault, [1] started, [2] a0 nonzero, [3] halted
//     (same LED order as the ARM boards; the ARM Z flag is replaced by "a0 nonzero").
//====================================================================
module top_cpu_riscv (
    input        clk,      // PIN_23
    input  [4:0] KEY,      // PIN_88,89,90,91,25 (only KEY[3], KEY[2] used)
    output [7:0] SEG,      // PIN_128,121,125,129,132,126,124,127
    output [3:0] DIG,      // PIN_133,135,136,137
    output [3:0] LEDG      // PIN_87,86,85,84
);

    //----------------------------------------------------------------
    // Clock divider: 50 MHz / 4 = 12.5 MHz for the CPU core
    //----------------------------------------------------------------
    reg [1:0] clkdiv = 2'b00;
    always @(posedge clk) clkdiv <= clkdiv + 2'b01;
    wire cpu_clk = clkdiv[1];

    //----------------------------------------------------------------
    // Reset button (KEY[3]), debounced level (active-low button)
    //----------------------------------------------------------------
    wire key3_raw = ~KEY[3];
    wire rst_debounced;
    debounce db_rst (.clk(clk), .btn_in(key3_raw), .btn_out(rst_debounced));

    //----------------------------------------------------------------
    // Run button (KEY[2]): first press sets 'started' until reset
    //----------------------------------------------------------------
    wire key2_raw = ~KEY[2];
    wire key2_debounced, step_pulse;
    debounce      db_step (.clk(cpu_clk), .btn_in(key2_raw), .btn_out(key2_debounced));
    edge_detector ed_step (.clk(cpu_clk), .signal_in(key2_debounced), .pulse_out(step_pulse));

    reg started = 1'b0;
    always @(posedge cpu_clk) begin
        if (rst_debounced)     started <= 1'b0;
        else if (step_pulse)   started <= 1'b1;
    end

    //----------------------------------------------------------------
    // CPU
    //----------------------------------------------------------------
    wire        snap_freeze, snap_override, snap_src;
    wire [5:0]  snap_idx;
    wire [31:0] snap_rf_data, snap_dm_data;
    wire        en_cpu = started & ~snap_freeze;        // CPU frozen while a snapshot runs

    // manual snapshot request (ISSP source, JTAG domain) -> 2 flip-flops -> drain
    reg drain_s1 = 1'b0, drain_s2 = 1'b0;
    always @(posedge cpu_clk) begin
        drain_s1 <= snap_src;
        drain_s2 <= drain_s1;
    end

    wire        halted, pipe_empty, illegal_flag, fault, ev_retire, ev_stall, ev_mstall, ev_redirect;
    wire [31:0] stop_pc;
    wire [31:0] pc_d_dbg, instr_d_dbg, x10_debug;
    wire [3:0]  valid_dbg;
    wire        branch_ex, taken_ex;

    cpu_top_pipe_riscv #(.ENABLE_M(1), .MUL_FAST(0)) mycpu (
        .clk          (cpu_clk),
        .rst          (rst_debounced),
        .en           (en_cpu),
        .drain        (drain_s2),
        .snap_override(snap_override),
        .snap_idx     (snap_idx),
        .snap_rf_data (snap_rf_data),
        .snap_dm_data (snap_dm_data),
        .halted       (halted),
        .pipe_empty   (pipe_empty),
        .illegal_flag (illegal_flag),
        .fault        (fault),
        .stop_pc      (stop_pc),
        .ev_retire    (ev_retire),
        .ev_stall     (ev_stall),
        .ev_mstall    (ev_mstall),
        .ev_redirect  (ev_redirect),
        .debug_pc_f   (),
        .debug_pc_d   (pc_d_dbg),
        .debug_instr_d(instr_d_dbg),
        .debug_pc_e   (),
        .debug_valid  (valid_dbg),
        .debug_branch_ex(branch_ex),
        .debug_taken  (taken_ex),
        .debug_x10    (x10_debug)
    );

    //----------------------------------------------------------------
    // Status helpers (cpu_clk, cleared by reset, count only enabled cycles)
    //----------------------------------------------------------------
    reg branch_taken_flag = 1'b0;                        // outcome of the last executed branch
    always @(posedge cpu_clk) begin
        if (rst_debounced)
            branch_taken_flag <= 1'b0;
        else if (en_cpu && branch_ex && valid_dbg[1])
            branch_taken_flag <= taken_ex;
    end

    // The measurement ends when the STOP instruction retires (its WB cycle is counted). 'halted' / 'fault' is set
    // at the end of that cycle, so the first cycle that sees it ends the count without counting itself.
    reg        done = 1'b0;
    reg [31:0] cycle_count = 32'd0, instr_count = 32'd0, stall_count = 32'd0, redirect_count = 32'd0;
    reg [15:0] mstall_count = 16'd0;
    always @(posedge cpu_clk) begin
        if (rst_debounced) begin
            done <= 1'b0;
            cycle_count <= 32'd0; instr_count <= 32'd0; stall_count <= 32'd0; redirect_count <= 32'd0; mstall_count <= 16'd0;
        end else if (halted | fault) begin
            done <= 1'b1;
        end else if (en_cpu && !done) begin
            cycle_count <= cycle_count + 32'd1;
            if (ev_retire)   instr_count    <= instr_count    + 32'd1;
            if (ev_stall)    stall_count    <= stall_count    + 32'd1;
            if (ev_redirect) redirect_count <= redirect_count + 32'd1;
            if (ev_mstall && mstall_count != 16'hFFFF) mstall_count <= mstall_count + 16'd1;
        end
    end

    //----------------------------------------------------------------
    // Snapshot unit + the four In-System Memory Content Editor RAMs
    //----------------------------------------------------------------
    snapshot_unit snap_unit (
        .clk          (cpu_clk),
        .rst          (rst_debounced),
        .started      (started),
        .trigger_auto (halted | fault),
        .trigger_src  (snap_src),
        .at_boundary  (pipe_empty),
        .rf_data      (snap_rf_data),
        .dm_data      (snap_dm_data),
        .snap_freeze  (snap_freeze),
        .snap_override(snap_override),
        .snap_idx     (snap_idx)
    );

    //----------------------------------------------------------------
    // 7-seg display removed: pins driven to constants
    //----------------------------------------------------------------
    assign SEG = 8'hFF;
    assign DIG = 4'hF;

    //----------------------------------------------------------------
    // Status LEDs (active-LOW: lit when the pin is driven 0)
    //----------------------------------------------------------------
    assign LEDG[0] = ~fault;
    assign LEDG[1] = ~started;
    assign LEDG[2] = ~(x10_debug != 32'd0);
    assign LEDG[3] = ~halted;

    //----------------------------------------------------------------
    // ISSP status probe (196 bits, same IP as the multi-cycle project). Source = manual snapshot request.
    //----------------------------------------------------------------
    wire [195:0] probe_status = {
        redirect_count,            // [195:164]
        stall_count,               // [163:132]
        instr_count,               // [131:100]
        cycle_count,               // [99:68]
        mstall_count,              // [67:52]
        (halted | fault) ? stop_pc[15:0] : pc_d_dbg[15:0],   // [51:36]
        instr_d_dbg,               // [35:4]
        fault, branch_taken_flag, halted, started   // [3:0]
    };
    cpu_issp_status inst_issp_status (.source(snap_src), .probe(probe_status));

endmodule
