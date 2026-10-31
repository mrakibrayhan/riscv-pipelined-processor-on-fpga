//====================================================================
// dmem_riscv_p.sv  (2026-10-24 Step E, gap G18) - same function as the old dmem_riscv.v of this core.
//   CHANGE: the 32-word RAM is now written with the Intel SystemVerilog byte-enable template
//   (logic [3:0][7:0], one write per byte lane), the same style as mem_arm_*.sv / mem_mips_*.sv,
//   so Quartus maps it to ONE M9K. The old 'reg [31:0]' array with part-select writes was built
//   from 1,024 flip-flops. The M9K output register has no power-up value (as in the ARM cores).
//   Module renamed dmem_riscv_p (was dmem_riscv) so the multi-cycle and pipelined files have unique names.
//====================================================================
//====================================================================
// dmem_riscv.v   (RISC-V 5-stage pipelined CPU, Step 6)
//
//   Data memory: one M9K array (default 32 words x 32 bit), synchronous read,
//   byte-enable writes (sb / sh / sw), sign/zero-extension for lb / lh / lbu / lhu / lw.
//
//   Pipeline timing (pipeline_spec.md section 2.1):
//     MEM stage, cycle t : addr = EX/MEM.alu_result, funct3 = EX/MEM.funct3,
//                          mem_read = EX/MEM.mem_read   (load)
//                          mem_write = EX/MEM.mem_write (store), write_data = EX/MEM.store_data
//     edge               : load  -> rdata_word, byte_off_r, funct3_r are captured
//                          store -> the selected bytes of the addressed word are written
//     WB stage, cycle t+1: read_data = sign/zero-extended load value (combinational from the
//                          captured word), used by the WB result mux and by forwarding.
//   So the M9K output register acts as the MEM/WB "ReadData" register. The lane (addr[1:0])
//   and funct3 are captured together with the word, because EX/MEM has moved on by then.
//
//   Contract with the CPU: mem_read and mem_write are gated with the global enable (en), so
//   nothing in here changes while the pipeline is paused. rdata_word/byte_off_r/funct3_r change
//   only on mem_read, so read_data stays valid until the next load.
//   (The snapshot unit overrides addr/funct3/mem_read outside this module, as before.)
//
//   Misaligned accesses are not trapped: half-word uses addr[1] only, word ignores addr[1:0].
//   Same data layout and addressing as the multi-cycle CPU (index = addr[ADDR_BITS+1:2]).
//====================================================================
module dmem_riscv_p #(
    parameter ADDR_BITS = 5,                 // 2**5 = 32 words
    parameter INIT_FILE = "data.hex"
)(
    input         clk,
    input  [31:0] addr,          // byte address (EX/MEM.alu_result)
    input  [31:0] write_data,    // raw rs2 value (EX/MEM.store_data)
    input  [2:0]  funct3,        // size / sign of the access (EX/MEM.funct3)
    input         mem_read,      // 1 in MEM for a load (gated with en)
    input         mem_write,     // 1 in MEM for a store (gated with en)
    output [31:0] read_data      // aligned + extended, valid in WB
);

    localparam DEPTH = (1 << ADDR_BITS);

    (* ramstyle = "M9K, no_rw_check" *) logic [3:0][7:0] ram [0:DEPTH-1];
    initial $readmemh(INIT_FILE, ram);

    wire [ADDR_BITS-1:0] word_idx = addr[ADDR_BITS+1:2];

    // ---------------- read side: captured at the end of MEM ----------------
    reg [31:0] rdata_word;
    reg [1:0]  byte_off_r;
    reg [2:0]  funct3_r;

    initial begin byte_off_r = 2'd0; funct3_r = 3'd0; end

    // (the M9K read is in the write block below: Intel byte-enable RAM template, one always block)
    always @(posedge clk) begin
        if (mem_read) begin
            byte_off_r <= addr[1:0];
            funct3_r   <= funct3;
        end
    end

    // ---------------- WB stage: align and extend ----------------
    wire [1:0] size        = funct3_r[1:0];   // 00 byte, 01 half, 10 word
    wire       unsigned_ld = funct3_r[2];

    wire [7:0]  sel_byte = rdata_word[byte_off_r*8 +: 8];
    wire [15:0] sel_half = byte_off_r[1] ? rdata_word[31:16] : rdata_word[15:0];

    reg [31:0] read_data_r;
    assign read_data = read_data_r;

    always @(*) begin
        case (size)
            2'b00:   read_data_r = unsigned_ld ? {24'b0, sel_byte} : {{24{sel_byte[7]}}, sel_byte};
            2'b01:   read_data_r = unsigned_ld ? {16'b0, sel_half} : {{16{sel_half[15]}}, sel_half};
            default: read_data_r = rdata_word;
        endcase
    end

    // ---------------- write side (MEM stage) ----------------
    wire [1:0] w_size = funct3[1:0];
    wire [1:0] w_off  = addr[1:0];

    reg [3:0]  byte_en;
    reg [31:0] write_word;

    always @(*) begin
        case (w_size)
            2'b00: begin                          // sb
                byte_en    = 4'b0001 << w_off;
                write_word = {4{write_data[7:0]}};
            end
            2'b01: begin                          // sh
                byte_en    = w_off[1] ? 4'b1100 : 4'b0011;
                write_word = {2{write_data[15:0]}};
            end
            default: begin                        // sw
                byte_en    = 4'b1111;
                write_word = write_data;
            end
        endcase
    end

    always @(posedge clk) begin
        if (mem_write) begin
            if (byte_en[0]) ram[word_idx][0]   <= write_word[7:0];
            if (byte_en[1]) ram[word_idx][1]  <= write_word[15:8];
            if (byte_en[2]) ram[word_idx][2] <= write_word[23:16];
            if (byte_en[3]) ram[word_idx][3] <= write_word[31:24];
        end
        if (mem_read)
            rdata_word <= ram[word_idx];             // M9K output register = [HH] ReadData / Data register
    end

endmodule