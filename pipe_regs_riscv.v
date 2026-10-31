//====================================================================
// pipe_regs_riscv.v   (RISC-V 5-stage pipelined CPU, Step 2)
//
//   The four pipeline registers of the textbook pipeline
//   (Harris & Harris, DDCA RISC-V Edition, Ch. 7), written as four small modules.
//
//   What is NOT in here (it lives inside the synchronous M9K memories, see
//   pipeline_spec.md section 2.1):
//       IF/ID.Instr   = imem_riscv.instruction
//       ID/EX.RD1/RD2 = regfile_riscv.q1 / q2
//       MEM/WB.ReadData = dmem_riscv.rdata_word
//
//   Rules shared by all four modules
//     * 'en' is the global run enable: when 0 every register holds.
//     * 'rst' is a synchronous reset of the CONTROL / VALID bits only.
//       Data fields (pc, imm, alu_result, ...) have no reset and no clear
//       (saves logic cells); a bubble is recognised by its control bits = 0.
//     * A "bubble" = valid 0 and every side-effect bit (reg_write, mem_read,
//       mem_write, branch, jump, sys, illegal) 0.
//
//   Control-word note: result_src of the spec (0 = ALU, 1 = load data) is
//   identical to mem_read (only loads select load data), so one bit is used:
//   mem_read = 1 means "load, result = load data".
//====================================================================


//--------------------------------------------------------------------
// IF/ID : pc and valid (the instruction itself is the IMEM output register)
//   hold      : keep the instruction in ID (load-use stall / EX hold)
//   flush     : redirect from EX, the new fetch is wrong-path -> valid = 0
//               (flush wins over hold)
//   no_fetch  : halt / draining: the instruction being fetched is not valid
//--------------------------------------------------------------------
module if_id_reg (
    input             clk,
    input             rst,
    input             en,
    input             hold,
    input             flush,
    input             no_fetch,
    input      [31:0] pc_in,        // PC of the instruction being fetched now
    output reg [31:0] pc,
    output reg        valid
);
    initial begin pc = 32'd0; valid = 1'b0; end

    always @(posedge clk) begin
        if (rst)
            valid <= 1'b0;
        else if (en) begin
            if (flush)         valid <= 1'b0;
            else if (!hold)    valid <= ~no_fetch;
        end
    end

    always @(posedge clk)
        if (en && !hold) pc <= pc_in;
endmodule


//--------------------------------------------------------------------
// ID/EX
//   hold   : keep everything (EX stalled by the multi-cycle M unit, Step 12)
//   bubble : load a bubble instead of the ID instruction
//            (load-use stall, redirect flush, halt). hold wins over bubble.
//--------------------------------------------------------------------
module id_ex_reg (
    input             clk,
    input             rst,
    input             en,
    input             hold,
    input             bubble,

    // from ID
    input             reg_write_d,
    input             mem_read_d,
    input             mem_write_d,
    input             branch_d,
    input             jump_d,
    input             jalr_d,
    input             sys_d,
    input             illegal_d,
    input             md_d,           // RV32M instruction (multi-cycle EX)
    input             valid_d,
    input      [1:0]  src_a_d,
    input      [1:0]  src_b_d,
    input      [3:0]  alu_ctrl_d,
    input      [2:0]  funct3_d,
    input      [31:0] pc_d,
    input      [31:0] imm_d,
    input      [4:0]  rs1_d,
    input      [4:0]  rs2_d,
    input      [4:0]  rd_d,

    // to EX
    output reg        reg_write_e,
    output reg        mem_read_e,
    output reg        mem_write_e,
    output reg        branch_e,
    output reg        jump_e,
    output reg        jalr_e,
    output reg        sys_e,
    output reg        illegal_e,
    output reg        md_e,
    output reg        valid_e,
    output reg [1:0]  src_a_e,
    output reg [1:0]  src_b_e,
    output reg [3:0]  alu_ctrl_e,
    output reg [2:0]  funct3_e,
    output reg [31:0] pc_e,
    output reg [31:0] imm_e,
    output reg [4:0]  rs1_e,
    output reg [4:0]  rs2_e,
    output reg [4:0]  rd_e
);
    initial begin
        reg_write_e = 0; mem_read_e = 0; mem_write_e = 0; branch_e = 0;
        jump_e = 0; jalr_e = 0; sys_e = 0; illegal_e = 0; md_e = 0; valid_e = 0;
        src_a_e = 0; src_b_e = 0; alu_ctrl_e = 0; funct3_e = 0;
        pc_e = 0; imm_e = 0; rs1_e = 0; rs2_e = 0; rd_e = 0;
    end

    wire load = en && !hold;

    // control / valid bits: reset and bubble
    always @(posedge clk) begin
        if (rst || (load && bubble)) begin
            reg_write_e <= 1'b0;
            mem_read_e  <= 1'b0;
            mem_write_e <= 1'b0;
            branch_e    <= 1'b0;
            jump_e      <= 1'b0;
            jalr_e      <= 1'b0;
            sys_e       <= 1'b0;
            illegal_e   <= 1'b0;
            md_e        <= 1'b0;
            valid_e     <= 1'b0;
        end else if (load) begin
            reg_write_e <= reg_write_d;
            mem_read_e  <= mem_read_d;
            mem_write_e <= mem_write_d;
            branch_e    <= branch_d;
            jump_e      <= jump_d;
            jalr_e      <= jalr_d;
            sys_e       <= sys_d;
            illegal_e   <= illegal_d;
            md_e        <= md_d;
            valid_e     <= valid_d;
        end
    end

    // data fields: plain enable, never cleared
    always @(posedge clk) begin
        if (load) begin
            src_a_e    <= src_a_d;
            src_b_e    <= src_b_d;
            alu_ctrl_e <= alu_ctrl_d;
            funct3_e   <= funct3_d;
            pc_e       <= pc_d;
            imm_e      <= imm_d;
            rs1_e      <= rs1_d;
            rs2_e      <= rs2_d;
            rd_e       <= rd_d;
        end
    end
endmodule


//--------------------------------------------------------------------
// EX/MEM
//   bubble : load a bubble (EX stalled by the M unit, Step 12; no new
//            instruction may leave EX while it is busy)
//--------------------------------------------------------------------
module ex_mem_reg (
    input             clk,
    input             rst,
    input             en,
    input             bubble,

    input             reg_write_e,
    input             mem_read_e,
    input             mem_write_e,
    input             valid_e,
    input      [2:0]  funct3_e,
    input      [31:0] alu_result_e,
    input      [31:0] store_data_e,
    input      [4:0]  rd_e,

    output reg        reg_write_m,
    output reg        mem_read_m,
    output reg        mem_write_m,
    output reg        valid_m,
    output reg [2:0]  funct3_m,
    output reg [31:0] alu_result_m,
    output reg [31:0] store_data_m,
    output reg [4:0]  rd_m
);
    initial begin
        reg_write_m = 0; mem_read_m = 0; mem_write_m = 0; valid_m = 0;
        funct3_m = 0; alu_result_m = 0; store_data_m = 0; rd_m = 0;
    end

    always @(posedge clk) begin
        if (rst || (en && bubble)) begin
            reg_write_m <= 1'b0;
            mem_read_m  <= 1'b0;
            mem_write_m <= 1'b0;
            valid_m     <= 1'b0;
        end else if (en) begin
            reg_write_m <= reg_write_e;
            mem_read_m  <= mem_read_e;
            mem_write_m <= mem_write_e;
            valid_m     <= valid_e;
        end
    end

    always @(posedge clk) begin
        if (en) begin
            funct3_m     <= funct3_e;
            alu_result_m <= alu_result_e;
            store_data_m <= store_data_e;
            rd_m         <= rd_e;
        end
    end
endmodule


//--------------------------------------------------------------------
// MEM/WB   (load data stays inside dmem_riscv; this holds the rest)
//--------------------------------------------------------------------
module mem_wb_reg (
    input             clk,
    input             rst,
    input             en,

    input             reg_write_m,
    input             mem_read_m,
    input             valid_m,
    input      [31:0] alu_result_m,
    input      [4:0]  rd_m,

    output reg        reg_write_w,
    output reg        mem_read_w,     // 1 = write back the load data
    output reg        valid_w,
    output reg [31:0] alu_result_w,
    output reg [4:0]  rd_w
);
    initial begin
        reg_write_w = 0; mem_read_w = 0; valid_w = 0; alu_result_w = 0; rd_w = 0;
    end

    always @(posedge clk) begin
        if (rst) begin
            reg_write_w <= 1'b0;
            mem_read_w  <= 1'b0;
            valid_w     <= 1'b0;
        end else if (en) begin
            reg_write_w <= reg_write_m;
            mem_read_w  <= mem_read_m;
            valid_w     <= valid_m;
        end
    end

    always @(posedge clk) begin
        if (en) begin
            alu_result_w <= alu_result_m;
            rd_w         <= rd_m;
        end
    end
endmodule