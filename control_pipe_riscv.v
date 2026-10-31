//====================================================================
// control_pipe_riscv.v   (RISC-V 5-stage pipelined CPU, Step 3)
//
//   Main decoder of the ID stage. Purely combinational: it turns the
//   instruction sitting in the IMEM output register (the IF/ID instruction)
//   into the control word that is stored in ID/EX (see pipeline_spec.md
//   sections 3 and 4).
//
//   * Every side-effect output is forced to 0 when 'valid' = 0, so a flushed
//     or empty IF/ID slot decodes to a bubble.
//   * reg_write is also forced to 0 when rd = x0, so no later stage needs an
//     x0 comparison (forwarding, hazard unit and regfile rely on this).
//   * Legality is decided on the opcode, exactly like the multi-cycle CPU:
//       - fence (any funct3) is a no-op,
//       - SYSTEM: only ecall (0x00000073) / ebreak (0x00100073) -> sys = 1 (halt),
//         every other SYSTEM encoding (CSR instructions) is illegal,
//       - an unknown opcode (including 0x00000000) is illegal: no side effects,
//         and 'illegal' is set (the sticky flag is set later, in EX).
//   * ENABLE_M = 0 : R-type with funct7 = 0000001 (RV32M) is illegal.
//   * ENABLE_M = 2 : Zmmul (Step E): mul / mulh / mulhsu / mulhu legal, div / divu / rem / remu illegal.
//     ENABLE_M = 1 : it is decoded as a multiply/divide: md_op = 1, reg_write = 1,
//                    no memory/branch/jump side effects (used from Step 12).
//
//   src_a : 00 rs1, 01 PC, 10 zero           src_b : 00 rs2, 01 imm, 10 four
//   imm_src (existing imm_gen): 000 I, 001 S, 010 B, 011 U, 100 J
//   alu_op  (existing alu_decoder_riscv): 00 add, 01 branch compare, 10 R-type, 11 I-type
//   alu_ctrl is produced here by alu_decoder_riscv (reused unchanged).
//====================================================================
module control_pipe_riscv #(
    parameter ENABLE_M = 0
)(
    input  [31:0] instr,
    input         valid,

    output        reg_write,
    output        mem_read,       // load: result = load data
    output        mem_write,      // store
    output        branch,
    output        jump,           // jal or jalr
    output        jalr,
    output        sys,            // ecall / ebreak
    output        illegal,
    output        md_op,          // RV32M instruction (only when ENABLE_M = 1)
    output        uses_rs1,
    output        uses_rs2,
    output [1:0]  src_a,
    output [1:0]  src_b,
    output [2:0]  imm_src,
    output [3:0]  alu_ctrl
);
    localparam OP_LOAD   = 7'b0000011;
    localparam OP_FENCE  = 7'b0001111;
    localparam OP_OPIMM  = 7'b0010011;
    localparam OP_AUIPC  = 7'b0010111;
    localparam OP_STORE  = 7'b0100011;
    localparam OP_OP     = 7'b0110011;
    localparam OP_LUI    = 7'b0110111;
    localparam OP_BRANCH = 7'b1100011;
    localparam OP_JALR   = 7'b1100111;
    localparam OP_JAL    = 7'b1101111;
    localparam OP_SYSTEM = 7'b1110011;

    wire [6:0] opcode = instr[6:0];
    wire [2:0] funct3 = instr[14:12];
    wire [4:0] rd     = instr[11:7];
    wire       is_md  = (instr[31:25] == 7'b0000001);
    wire       sys_op = (instr == 32'h00000073) || (instr == 32'h00100073);

    reg        r_rw, r_mr, r_mw, r_br, r_jp, r_jr, r_sys, r_ill, r_md, r_u1, r_u2;
    reg [1:0]  r_sa, r_sb, r_aop;
    reg [2:0]  r_imm;

    always @(*) begin
        r_rw = 0; r_mr = 0; r_mw = 0; r_br = 0; r_jp = 0; r_jr = 0; r_sys = 0; r_ill = 0; r_md = 0;
        r_u1 = 0; r_u2 = 0;
        r_sa = 2'b00; r_sb = 2'b00; r_aop = 2'b00; r_imm = 3'b000;

        case (opcode)
            OP_OP: begin
                if (is_md && ((ENABLE_M == 0) || ((ENABLE_M == 2) && funct3[2]))) r_ill = 1'b1;   // ENABLE_M 2 = Zmmul (Step E)
                else begin
                    r_rw = 1; r_u1 = 1; r_u2 = 1;
                    r_sa = 2'b00; r_sb = 2'b00; r_aop = 2'b10;
                    r_md = is_md;
                end
            end
            OP_OPIMM: begin
                r_rw = 1; r_u1 = 1;
                r_sa = 2'b00; r_sb = 2'b01; r_aop = 2'b11; r_imm = 3'b000;
            end
            OP_LOAD: begin
                r_rw = 1; r_mr = 1; r_u1 = 1;
                r_sa = 2'b00; r_sb = 2'b01; r_aop = 2'b00; r_imm = 3'b000;
            end
            OP_STORE: begin
                r_mw = 1; r_u1 = 1; r_u2 = 1;
                r_sa = 2'b00; r_sb = 2'b01; r_aop = 2'b00; r_imm = 3'b001;
            end
            OP_BRANCH: begin
                r_br = 1; r_u1 = 1; r_u2 = 1;
                r_sa = 2'b00; r_sb = 2'b00; r_aop = 2'b01; r_imm = 3'b010;
            end
            OP_JAL: begin
                r_rw = 1; r_jp = 1;
                r_sa = 2'b01; r_sb = 2'b10; r_aop = 2'b00; r_imm = 3'b100;   // link = pc + 4 from the ALU
            end
            OP_JALR: begin
                r_rw = 1; r_jp = 1; r_jr = 1; r_u1 = 1;
                r_sa = 2'b01; r_sb = 2'b10; r_aop = 2'b00; r_imm = 3'b000;
            end
            OP_LUI: begin
                r_rw = 1;
                r_sa = 2'b10; r_sb = 2'b01; r_aop = 2'b00; r_imm = 3'b011;   // 0 + imm
            end
            OP_AUIPC: begin
                r_rw = 1;
                r_sa = 2'b01; r_sb = 2'b01; r_aop = 2'b00; r_imm = 3'b011;   // pc + imm
            end
            OP_FENCE: begin
                // no-op
            end
            OP_SYSTEM: begin
                if (sys_op) r_sys = 1'b1;
                else        r_ill = 1'b1;                                     // CSR instructions
            end
            default: r_ill = 1'b1;
        endcase
    end

    assign reg_write = valid & r_rw & (rd != 5'd0);
    assign mem_read  = valid & r_mr;
    assign mem_write = valid & r_mw;
    assign branch    = valid & r_br;
    assign jump      = valid & r_jp;
    assign jalr      = valid & r_jr;
    assign sys       = valid & r_sys;
    assign illegal   = valid & r_ill;
    assign md_op     = valid & r_md;
    assign uses_rs1  = valid & r_u1;
    assign uses_rs2  = valid & r_u2;

    assign src_a   = r_sa;
    assign src_b   = r_sb;
    assign imm_src = r_imm;

    alu_decoder_riscv u_aludec (
        .alu_op   (r_aop),
        .funct3   (funct3),
        .funct7b5 (instr[30]),
        .alu_ctrl (alu_ctrl)
    );
endmodule