//====================================================================
// if_stage_riscv.v   (RISC-V 5-stage pipelined CPU, Step 4)
//
//   IF stage = PC register + PC+4 + synchronous M9K instruction memory
//              + the IF/ID register (pc, valid).
//
//   Timing (pipeline_spec.md section 2.1):
//     cycle t   : PC = A, imem is addressed with A, pc_f = A
//     edge      : instr_d <= mem[A], pc_d <= A, valid_d <= 1 (unless killed)
//     cycle t+1 : the instruction at A is in ID: instr_d, pc_d, valid_d
//   So the IMEM output register IS the IF/ID instruction register.
//
//   Controls (all from the hazard unit / EX stage):
//     hold     : load-use stall (or EX hold for RV32M): PC, IF/ID and the IMEM
//                output register keep their values.
//     redirect : taken branch / jal / jalr in EX: PC <= target, the instruction
//                arriving in IF/ID is wrong-path -> valid_d = 0
//                (the instruction in ID is killed by the ID/EX bubble).
//     no_fetch : halted or draining: PC is held, the arriving instruction is not
//                marked valid. A redirect from an older instruction still wins.
//                The PC then holds the address of the next instruction that has
//                NOT entered the pipeline, so fetch can resume without losing or
//                repeating an instruction.
//     en       : global run enable, everything holds when 0.
//   Priority in PC: redirect > (hold | no_fetch) > PC+4.
//====================================================================
module if_stage_riscv #(
    parameter ADDR_BITS = 12,                  // imem depth = 2**ADDR_BITS words
    parameter INIT_FILE = "program.hex",
    parameter [31:0] RESET_PC = 32'd0
)(
    input         clk,
    input         rst,
    input         en,
    input         hold,
    input         redirect,
    input  [31:0] target,
    input         no_fetch,

    output [31:0] pc_f,        // address being fetched in this cycle
    output [31:0] instr_d,     // instruction in ID (IMEM output register)
    output [31:0] pc_d,        // its address
    output        valid_d      // 0 = bubble (flushed / not fetched)
);
    reg [31:0] pc;
    initial pc = RESET_PC;

    always @(posedge clk) begin
        if (rst)
            pc <= RESET_PC;
        else if (en) begin
            if (redirect)                  pc <= target;
            else if (!hold && !no_fetch)   pc <= pc + 32'd4;
        end
    end
    assign pc_f = pc;

    imem_riscv #(.ADDR_BITS(ADDR_BITS), .INIT_FILE(INIT_FILE)) u_imem (
        .clk        (clk),
        .fetch_en   (en & ~hold),
        .pc         (pc),
        .instruction(instr_d)
    );

    if_id_reg u_ifid (
        .clk      (clk),
        .rst      (rst),
        .en       (en),
        .hold     (hold),
        .flush    (redirect),
        .no_fetch (no_fetch),
        .pc_in    (pc),
        .pc       (pc_d),
        .valid    (valid_d)
    );
endmodule