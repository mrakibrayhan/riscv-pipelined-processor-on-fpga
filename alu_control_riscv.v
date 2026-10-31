//====================================================================
// alu_control_riscv.v   (RISC-V RV32I multi-cycle CPU - step 3)
//
//   Reference: Harris & Harris, "Digital Design and Computer
//   Architecture: RISC-V Edition", Ch. 7 (ALU decoder), extended to
//   the full RV32I set. See multicycle_spec.md Sections 2.5 and 4.
//
//   Two small combinational modules. The ALU itself (alu_riscv.v +
//   ALU_32_bit.v) is reused UNCHANGED from the single-cycle project;
//   its alu_ctrl encoding is kept so both CPUs share one ALU.
//
//   1) alu_decoder_riscv : ALUOp (from the FSM) + funct3 + instr[30]
//                          -> alu_ctrl for alu_riscv
//   2) branch_cond_riscv : funct3 + ALU flags -> branch 'taken'
//
//   ALUOp encoding (driven by the FSM, spec Section 4):
//       2'b00  force ADD
//       2'b01  branch compare (SUB / SLT / SLTU chosen from funct3)
//       2'b10  R-type  (funct3, funct7[5])
//       2'b11  I-type  (funct3; funct7[5] used ONLY for shifts)
//
//   Why I-type is its own ALUOp: for R-type, funct7[5]=1 with
//   funct3=000 means SUB. For I-type the same bit is just the sign of
//   the immediate (addi x1,x1,-1 has instr[30]=1), so addi must stay
//   an ADD. Only srai (funct3=101) uses instr[30] in I-type.
//====================================================================

module alu_decoder_riscv (
    input      [1:0] alu_op,
    input      [2:0] funct3,
    input            funct7b5,      // instr[30]
    output reg [3:0] alu_ctrl
);
    // ---- must match alu_riscv.v / control_unit_riscv.v exactly ----
    localparam ALU_ADD  = 4'b0000;
    localparam ALU_SUB  = 4'b0001;
    localparam ALU_AND  = 4'b0010;
    localparam ALU_OR   = 4'b0011;
    localparam ALU_XOR  = 4'b0100;
    localparam ALU_SLL  = 4'b0101;
    localparam ALU_SRL  = 4'b0110;
    localparam ALU_SRA  = 4'b0111;
    localparam ALU_SLT  = 4'b1000;
    localparam ALU_SLTU = 4'b1001;

    always @(*) begin
        case (alu_op)
            2'b00: alu_ctrl = ALU_ADD;

            2'b01: begin                         // branch compare
                case (funct3)
                    3'b000, 3'b001: alu_ctrl = ALU_SUB;   // beq, bne
                    3'b100, 3'b101: alu_ctrl = ALU_SLT;   // blt, bge
                    3'b110, 3'b111: alu_ctrl = ALU_SLTU;  // bltu, bgeu
                    default:        alu_ctrl = ALU_SUB;
                endcase
            end

            2'b10: begin                         // R-type
                case (funct3)
                    3'b000:  alu_ctrl = funct7b5 ? ALU_SUB : ALU_ADD;
                    3'b001:  alu_ctrl = ALU_SLL;
                    3'b010:  alu_ctrl = ALU_SLT;
                    3'b011:  alu_ctrl = ALU_SLTU;
                    3'b100:  alu_ctrl = ALU_XOR;
                    3'b101:  alu_ctrl = funct7b5 ? ALU_SRA : ALU_SRL;
                    3'b110:  alu_ctrl = ALU_OR;
                    default: alu_ctrl = ALU_AND;          // 3'b111
                endcase
            end

            default: begin                       // 2'b11: I-type
                case (funct3)
                    3'b000:  alu_ctrl = ALU_ADD;          // addi: never SUB
                    3'b001:  alu_ctrl = ALU_SLL;          // slli
                    3'b010:  alu_ctrl = ALU_SLT;          // slti
                    3'b011:  alu_ctrl = ALU_SLTU;         // sltiu
                    3'b100:  alu_ctrl = ALU_XOR;          // xori
                    3'b101:  alu_ctrl = funct7b5 ? ALU_SRA : ALU_SRL; // srai / srli
                    3'b110:  alu_ctrl = ALU_OR;           // ori
                    default: alu_ctrl = ALU_AND;          // andi
                endcase
            end
        endcase
    end
endmodule


//====================================================================
// branch_cond_riscv: is the branch condition true?
//
//   funct3  inst   ALU op   test
//   000     beq    SUB      zero
//   001     bne    SUB      ~zero
//   100     blt    SLT      result[0]
//   101     bge    SLT      ~result[0]
//   110     bltu   SLTU     result[0]
//   111     bgeu   SLTU     ~result[0]
//
//   funct3[2] picks zero vs. result[0]; funct3[0] inverts.
//   The FSM combines it as:  PCUpdate = PCWrite | (Branch & taken)
//   funct3 = 010 / 011 are not branches (decode marks them illegal).
//====================================================================
module branch_cond_riscv (
    input  [2:0] funct3,
    input        zero_flag,       // from alu_riscv
    input        alu_result_lsb,  // alu_result[0]
    output       taken
);
    assign taken = (funct3[2] ? alu_result_lsb : zero_flag) ^ funct3[0];
endmodule