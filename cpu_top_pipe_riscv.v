//====================================================================
// cpu_top_pipe_riscv.v   (RISC-V 5-stage pipelined CPU, Step 8: datapath assembly)
//
//   RV32I (40 instructions incl. fence / ecall / ebreak), synchronous-read M9K memories.
//   Follows pipeline_spec.md (Harris & Harris, DDCA RISC-V Edition, Ch. 7).
//
//     IF  : if_stage_riscv   PC, PC+4, imem (M9K), IF/ID (pc, valid)
//     ID  : control_pipe_riscv, imm_gen, regfile_riscv (M9K x2, sync read, write-first bypass)
//           -> ID/EX (id_ex_reg); regfile output = A / B
//     EX  : forwarding muxes, SrcA / SrcB muxes, alu_riscv, branch_cond_riscv,
//           target adder, redirect  -> EX/MEM (ex_mem_reg)
//     MEM : dmem_riscv (M9K, sync read) -> MEM/WB (mem_wb_reg); load data stays inside dmem
//     WB  : result mux (ALU / load data) -> register file write port, forwarding source
//     hazard_unit_riscv: forwarding selects, load-use stall, redirect, halt, drain
//
//   Global run enable 'en' (= started and not frozen by the snapshot unit): when 0 every state
//   register holds, memory writes / reads are gated, the register write is gated.
//
//   Snapshot interface (same idea as the multi-cycle CPU, see snapshot_unit.v): while
//   snap_override is high (the pipeline is empty and en = 0) regfile read port 1 and the dmem
//   read address are driven by snap_idx. Data returns one cycle later.
//   'drain' (manual snapshot request) stops fetching; the PC then holds the next instruction that has
//   not entered the pipeline, so execution resumes without losing or repeating an instruction.
//   'pipe_empty' = no valid instruction in ID, EX, MEM, WB (snapshot_unit 'at_boundary').
//
//   Stop (same rule as the ARM pipeline, arm_p_spec.md): a STOP instruction in EX kills everything
//   younger and stops fetching ('stopping'); older instructions finish. When the stop instruction
//   reaches WB, 'halted' or 'fault' is set (sticky until rst).
//     halt  : ecall, ebreak, or a taken branch / jal whose target is itself ("done: beq x2,x2,done",
//             the last line of the Harris & Harris test program riscvtest.s). jal still writes rd.
//     fault : an illegal encoding (unknown opcode, CSR instruction, RV32M when ENABLE_M = 0).
//   stop_pc = address of the stop instruction for a branch / jal to itself, else address + 4
//   (the ARM convention: "B ." -> its own address, SWI / fault -> address + 4).
//   fence is a no-op. No traps; misaligned accesses are not trapped.
//
//   Event strobes for the debug counters (kept outside the core so core logic can be measured
//   separately): ev_retire (valid instruction in WB), ev_stall (load-use stall), ev_redirect.
//====================================================================
module cpu_top_pipe_riscv #(
    parameter MUL_FAST = 0,              // 0 = serial shift-add multiply, 1 = embedded-multiplier multiply (see muldiv_riscv.v)
    parameter ENABLE_M = 1,              // 1 = RV32IM (iterative muldiv unit in EX), 0 = RV32I, 2 = RV32I_Zmmul (multiply only; Step E)
    parameter IMEM_AW  = 7,              // instruction ROM depth 2**IMEM_AW words: 7 = 128 words (default, as all other cores), 12 = 4096 words
    parameter DMEM_AW  = 5               // data RAM depth 2**DMEM_AW words: 5 = 32 words (board / CPU-only default; Step F simulation uses more)
)(
    input         clk,
    input         rst,                 // hold >= 32 cycles (register-file clear counter)
    input         en,                  // global run enable

    input         drain,               // manual snapshot request: stop fetching
    input         snap_override,
    input  [5:0]  snap_idx,
    output [31:0] snap_rf_data,
    output [31:0] snap_dm_data,

    output        halted,
    output        pipe_empty,
    output        illegal_flag,        // = fault (sticky): an illegal instruction stopped the CPU
    output        fault,
    output [31:0] stop_pc,
    output        ev_retire,
    output        ev_stall,            // load-use stall cycle
    output        ev_mstall,           // EX stalled by the M unit (RV32M)
    output        ev_redirect,

    // debug taps
    output [31:0] debug_pc_f,          // address being fetched
    output [31:0] debug_pc_d,          // instruction in ID: address and word
    output [31:0] debug_instr_d,
    output [31:0] debug_pc_e,          // instruction in EX: address
    output [3:0]  debug_valid,         // {WB, MEM, EX, ID} valid bits
    output        debug_branch_ex,     // a conditional branch is in EX
    output        debug_taken,         // branch condition result (meaningful with debug_branch_ex)
    output [31:0] debug_x10            // a0 mirror (LED)
);

    //================================================================
    // IF
    //================================================================
    wire        hold_front, redirect, no_fetch;
    wire [31:0] target;
    wire [31:0] instr_d, pc_d, pc_f;
    wire        valid_d;

    if_stage_riscv #(.ADDR_BITS(IMEM_AW), .INIT_FILE("program.hex")) u_if (
        .clk(clk), .rst(rst), .en(en),
        .hold(hold_front), .redirect(redirect), .target(target), .no_fetch(no_fetch),
        .pc_f(pc_f), .instr_d(instr_d), .pc_d(pc_d), .valid_d(valid_d)
    );

    //================================================================
    // ID
    //================================================================
    wire        c_reg_write, c_mem_read, c_mem_write, c_branch, c_jump, c_jalr, c_sys, c_illegal, c_md;
    wire        c_uses_rs1, c_uses_rs2;
    wire [1:0]  c_src_a, c_src_b;
    wire [2:0]  c_imm_src;
    wire [3:0]  c_alu_ctrl;

    control_pipe_riscv #(.ENABLE_M(ENABLE_M)) u_ctl (
        .instr(instr_d), .valid(valid_d),
        .reg_write(c_reg_write), .mem_read(c_mem_read), .mem_write(c_mem_write),
        .branch(c_branch), .jump(c_jump), .jalr(c_jalr), .sys(c_sys), .illegal(c_illegal),
        .md_op(c_md), .uses_rs1(c_uses_rs1), .uses_rs2(c_uses_rs2),
        .src_a(c_src_a), .src_b(c_src_b), .imm_src(c_imm_src), .alu_ctrl(c_alu_ctrl)
    );

    wire [31:0] imm_d;
    imm_gen u_imm (.instruction(instr_d), .imm_src(c_imm_src), .imm_out(imm_d));

    wire [4:0] rs1_d = instr_d[19:15];
    wire [4:0] rs2_d = instr_d[24:20];
    wire [4:0] rd_d  = instr_d[11:7];

    // register file (write port is driven from WB, see below)
    wire [31:0] rf_a, rf_b;
    wire [4:0]  rd_w;
    wire        reg_write_w, mem_read_w;
    wire [31:0] alu_result_w, load_data_w;
    wire [31:0] wb_result = mem_read_w ? load_data_w : alu_result_w;

    regfile_riscv u_rf (
        .clk(clk), .rst(rst), .read_en(en | snap_override),
        .read_reg1(snap_override ? snap_idx[4:0] : rs1_d), .read_reg2(rs2_d),
        .write_reg(rd_w), .write_data(wb_result), .write_enable(reg_write_w & en),
        .read_data1(rf_a), .read_data2(rf_b), .dbg_x10(debug_x10)
    );

    //================================================================
    // ID/EX
    //================================================================
    wire        bubble_id_ex, hold_id_ex, bubble_ex_mem;
    wire        reg_write_e, mem_read_e, mem_write_e, branch_e, jump_e, jalr_e, sys_e, illegal_e, md_e, valid_e;
    wire [1:0]  src_a_e, src_b_e;
    wire [3:0]  alu_ctrl_e;
    wire [2:0]  funct3_e;
    wire [31:0] pc_e, imm_e;
    wire [4:0]  rs1_e, rs2_e, rd_e;

    id_ex_reg u_idex (
        .clk(clk), .rst(rst), .en(en), .hold(hold_id_ex), .bubble(bubble_id_ex),
        .reg_write_d(c_reg_write), .mem_read_d(c_mem_read), .mem_write_d(c_mem_write),
        .branch_d(c_branch), .jump_d(c_jump), .jalr_d(c_jalr), .sys_d(c_sys), .illegal_d(c_illegal), .md_d(c_md),
        .valid_d(valid_d), .src_a_d(c_src_a), .src_b_d(c_src_b), .alu_ctrl_d(c_alu_ctrl),
        .funct3_d(instr_d[14:12]), .pc_d(pc_d), .imm_d(imm_d), .rs1_d(rs1_d), .rs2_d(rs2_d), .rd_d(rd_d),
        .reg_write_e(reg_write_e), .mem_read_e(mem_read_e), .mem_write_e(mem_write_e),
        .branch_e(branch_e), .jump_e(jump_e), .jalr_e(jalr_e), .sys_e(sys_e), .illegal_e(illegal_e), .md_e(md_e),
        .valid_e(valid_e), .src_a_e(src_a_e), .src_b_e(src_b_e), .alu_ctrl_e(alu_ctrl_e),
        .funct3_e(funct3_e), .pc_e(pc_e), .imm_e(imm_e), .rs1_e(rs1_e), .rs2_e(rs2_e), .rd_e(rd_e)
    );

    //================================================================
    // EX
    //================================================================
    wire [1:0]  forward_a, forward_b;
    wire [31:0] alu_result_m;                       // EX/MEM
    wire        reg_write_m, mem_read_m, mem_write_m, valid_m;
    wire [2:0]  funct3_m;
    wire [31:0] store_data_m;
    wire [4:0]  rd_m;
    reg         halted_r = 1'b0, illegal_r = 1'b0, stopping_r = 1'b0;
    wire [31:0] op_a = (forward_a == 2'b10) ? alu_result_m :
                       (forward_a == 2'b01) ? wb_result    : rf_a;
    wire [31:0] op_b = (forward_b == 2'b10) ? alu_result_m :
                       (forward_b == 2'b01) ? wb_result    : rf_b;

    wire [31:0] src_a = (src_a_e == 2'b00) ? op_a :
                        (src_a_e == 2'b01) ? pc_e : 32'd0;
    wire [31:0] src_b = (src_b_e == 2'b00) ? op_b :
                        (src_b_e == 2'b01) ? imm_e : 32'd4;

    wire [31:0] alu_result_e;
    wire        alu_zero, taken;
    alu_riscv u_alu (.A(src_a), .B(src_b), .alu_ctrl(alu_ctrl_e), .alu_result(alu_result_e), .zero_flag(alu_zero));
    branch_cond_riscv u_bc (.funct3(funct3_e), .zero_flag(alu_zero), .alu_result_lsb(alu_result_e[0]), .taken(taken));

    // branch / jal / jalr target
    wire [31:0] tgt_sum = (jalr_e ? op_a : pc_e) + imm_e;
    assign target = {tgt_sum[31:1], tgt_sum[0] & ~jalr_e};

    // ---------------- RV32M: iterative unit, EX stalls until it is done ----------------
    //   first EX cycle : md_first = 1 -> 'start' (the unit latches rs1 / rs2 / funct3)
    //   later cycles   : md_active = 1, EX stays stalled until the unit's 'done' level is seen
    //   done cycle     : md_fin = 1 -> EX is released, the unit result goes down the ALU-result path
    wire        ex_stall;
    wire [31:0] md_result;
    generate if (ENABLE_M) begin : g_m
        reg  md_active = 1'b0;
        wire md_done;
        wire md_first = md_e & ~md_active;
        wire md_fin   = md_active & md_done;
        always @(posedge clk) begin
            if (rst) md_active <= 1'b0;
            else if (en) begin
                if (md_first)    md_active <= 1'b1;
                else if (md_fin) md_active <= 1'b0;
            end
        end
        muldiv_riscv #(.MUL_FAST(MUL_FAST), .HAS_DIV(ENABLE_M == 1)) u_md (.clk(clk), .rst(rst), .en(en), .start(md_first), .funct3(funct3_e),
                           .a(op_a), .b(op_b), .done(md_done), .result(md_result));
        assign ex_stall = md_e & ~md_fin;
    end else begin : g_nom
        assign ex_stall  = 1'b0;
        assign md_result = 32'd0;
    end endgenerate
    wire [31:0] ex_result = md_e ? md_result : alu_result_e;

    wire stall_ld, halt_now, stop_e;

    hazard_unit_riscv u_hz (
        .rs1_d(rs1_d), .rs2_d(rs2_d), .uses_rs1_d(c_uses_rs1), .uses_rs2_d(c_uses_rs2),
        .rs1_e(rs1_e), .rs2_e(rs2_e), .rd_e(rd_e), .reg_write_e(reg_write_e), .mem_read_e(mem_read_e),
        .stop_e(stop_e), .branch_e(branch_e), .jump_e(jump_e), .taken(taken), .ex_stall(ex_stall),
        .rd_m(rd_m), .reg_write_m(reg_write_m), .rd_w(rd_w), .reg_write_w(reg_write_w),
        .stopping(stopping_r), .drain(drain),
        .forward_a(forward_a), .forward_b(forward_b), .stall_ld(stall_ld), .redirect(redirect),
        .halt_now(halt_now), .hold_front(hold_front), .no_fetch(no_fetch),
        .bubble_id_ex(bubble_id_ex), .hold_id_ex(hold_id_ex), .bubble_ex_mem(bubble_ex_mem)
    );

    // ---------------- stop: detected in EX, takes effect in WB ----------------
    wire self_loop = ((jump_e & ~jalr_e) | (branch_e & taken)) & (imm_e == 32'd0);
    assign stop_e  = valid_e & (sys_e | illegal_e | self_loop);
    reg  stop_m = 1'b0, fault_m = 1'b0, stop_w = 1'b0, fault_w = 1'b0;
    reg  [31:0] stop_pc_r = 32'd0;
    always @(posedge clk) begin
        if (rst) begin
            stopping_r <= 1'b0; stop_m <= 1'b0; fault_m <= 1'b0; stop_w <= 1'b0; fault_w <= 1'b0;
            halted_r <= 1'b0; illegal_r <= 1'b0; stop_pc_r <= 32'd0;
        end else if (en) begin
            if (stop_e & ~stopping_r) begin stopping_r <= 1'b1; stop_pc_r <= self_loop ? pc_e : pc_e + 32'd4; end
            stop_m  <= stop_e & ~stopping_r;  fault_m <= stop_e & ~stopping_r & illegal_e;   // EX/MEM
            stop_w  <= stop_m;                fault_w <= fault_m;            // MEM/WB
            if (stop_w &  fault_w) illegal_r <= 1'b1;                        // the stop instruction is in WB
            if (stop_w & ~fault_w) halted_r  <= 1'b1;
        end
    end
    assign halted       = halted_r;
    assign fault        = illegal_r;
    assign illegal_flag = illegal_r;
    assign stop_pc      = stop_pc_r;

    //================================================================
    // EX/MEM
    //================================================================

    ex_mem_reg u_exmem (
        .clk(clk), .rst(rst), .en(en), .bubble(bubble_ex_mem),
        .reg_write_e(reg_write_e), .mem_read_e(mem_read_e), .mem_write_e(mem_write_e), .valid_e(valid_e),
        .funct3_e(funct3_e), .alu_result_e(ex_result), .store_data_e(op_b), .rd_e(rd_e),
        .reg_write_m(reg_write_m), .mem_read_m(mem_read_m), .mem_write_m(mem_write_m), .valid_m(valid_m),
        .funct3_m(funct3_m), .alu_result_m(alu_result_m), .store_data_m(store_data_m), .rd_m(rd_m)
    );

    //================================================================
    // MEM
    //================================================================
    wire [31:0] dm_addr   = snap_override ? {25'd0, snap_idx[4:0], 2'b00} : alu_result_m;
    wire [2:0]  dm_funct3 = snap_override ? 3'b010 : funct3_m;

    dmem_riscv_p #(.ADDR_BITS(DMEM_AW), .INIT_FILE("data.hex")) u_dmem (
        .clk(clk), .addr(dm_addr), .write_data(store_data_m), .funct3(dm_funct3),
        .mem_read((mem_read_m & en) | snap_override),
        .mem_write(mem_write_m & en & ~snap_override),
        .read_data(load_data_w)
    );

    //================================================================
    // MEM/WB
    //================================================================
    wire valid_w;
    mem_wb_reg u_memwb (
        .clk(clk), .rst(rst), .en(en),
        .reg_write_m(reg_write_m), .mem_read_m(mem_read_m), .valid_m(valid_m),
        .alu_result_m(alu_result_m), .rd_m(rd_m),
        .reg_write_w(reg_write_w), .mem_read_w(mem_read_w), .valid_w(valid_w),
        .alu_result_w(alu_result_w), .rd_w(rd_w)
    );

    //================================================================
    // Snapshot / debug / events
    //================================================================
    assign snap_rf_data = rf_a;
    assign snap_dm_data = load_data_w;
    assign pipe_empty   = ~(valid_d | valid_e | valid_m | valid_w);

    assign ev_retire   = valid_w & en;
    assign ev_stall    = stall_ld & en;
    assign ev_redirect = redirect & en;
    assign ev_mstall   = ex_stall & en;

    assign debug_pc_f      = pc_f;
    assign debug_pc_d      = pc_d;
    assign debug_instr_d   = instr_d;
    assign debug_pc_e      = pc_e;
    assign debug_valid     = {valid_w, valid_m, valid_e, valid_d};
    assign debug_branch_ex = branch_e;
    assign debug_taken     = taken;

endmodule
