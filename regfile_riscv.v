//====================================================================
// regfile_riscv.v   (RISC-V 5-stage pipelined CPU, Step 5)
//
//   Two identical M9K copies (ram1 for rs1, ram2 for rs2), both written together.
//   Both READ PORTS ARE SYNCHRONOUS: the address (rs1/rs2 field of the instruction in ID)
//   is captured at the clock edge that ends ID and read_data1/read_data2 are valid in
//   the next cycle (EX). The outputs are used directly as the ID/EX A and B registers.
//
//   NEW in the pipelined version
//   1. read_en : clock enable of the read ports (q1, q2) and of the bypass registers.
//      When 0 the outputs hold. The CPU drives it with  (en | snapshot_override).
//   2. WRITE-FIRST BYPASS (WB -> ID): if the instruction in WB writes register r at the
//      same clock edge at which the instruction in ID reads r, the plain M9K returns
//      the OLD value. Here the NEW value (write_data) is returned instead:
//          read_data = (write at the same edge to the same non-zero register) ? write_data
//                                                                              : ram[addr]
//      Cost: 32 flip-flops (wd_r) + 2 flags + a 2:1 mux per port. Needed because M9K
//      cannot be written on the falling clock edge (textbook trick) at one clock.
//
//   Contract with the CPU: write_enable must be 0 whenever read_en is 0 (the CPU gates the
//   WB write with the same global enable). This keeps the behaviour independent of how
//   the fitter implements the M9K read hold. (Simulation-only check below.)
//
//   x0: never written (write_reg = 0 is ignored), both copies are cleared to 0 -> reads 0.
//   Reset: the two RAM copies are cleared by a 5-bit counter that writes zeros to all 32
//   addresses while rst is high, so rst must stay high for at least 32 clock cycles.
//   Only x10 (a0) is mirrored in 32 flip-flops, for the "a0 nonzero" LED.
//====================================================================
module regfile_riscv (
    input         clk,
    input         rst,          // synchronous; hold >= 32 cycles (clears the RAM copies)
    input         read_en,      // clock enable of the read side
    input  [4:0]  read_reg1,    // rs1 (or the snapshot index)
    input  [4:0]  read_reg2,    // rs2
    input  [4:0]  write_reg,    // rd  (from WB)
    input  [31:0] write_data,   // result (from WB)
    input         write_enable, // reg_write (from WB), already gated with the global enable
    output [31:0] read_data1,   // synchronous: valid one cycle after read_reg1 is captured
    output [31:0] read_data2,
    output [31:0] dbg_x10       // a0 mirror (LED only)
);

    reg [4:0] clr_cnt = 5'd0;                       // runs while rst is high
    always @(posedge clk) begin
        if (rst) clr_cnt <= clr_cnt + 5'd1;
        else     clr_cnt <= 5'd0;
    end

    wire        wr_ok     = write_enable & (write_reg != 5'd0);
    wire        ram_we    = rst | wr_ok;
    wire [4:0]  ram_waddr = rst ? clr_cnt : write_reg;
    wire [31:0] ram_wdata = rst ? 32'd0   : write_data;

    (* ramstyle = "M9K, no_rw_check" *) reg [31:0] ram1 [0:31];
    (* ramstyle = "M9K, no_rw_check" *) reg [31:0] ram2 [0:31];
    reg [31:0] q1, q2;

    integer k;
    initial begin
        for (k = 0; k < 32; k = k + 1) begin ram1[k] = 32'd0; ram2[k] = 32'd0; end
        q1 = 32'd0; q2 = 32'd0;
    end

    always @(posedge clk) begin
        if (ram_we) ram1[ram_waddr] <= ram_wdata;
        if (read_en) q1 <= ram1[read_reg1];
    end
    always @(posedge clk) begin
        if (ram_we) ram2[ram_waddr] <= ram_wdata;
        if (read_en) q2 <= ram2[read_reg2];
    end

    // ---------------- write-first bypass ----------------
    reg [31:0] wd_r;
    reg        byp1, byp2;
    initial begin wd_r = 32'd0; byp1 = 1'b0; byp2 = 1'b0; end

    always @(posedge clk) begin
        if (read_en) begin
            wd_r <= write_data;
            byp1 <= ~rst & wr_ok & (write_reg == read_reg1);
            byp2 <= ~rst & wr_ok & (write_reg == read_reg2);
        end
    end

    assign read_data1 = byp1 ? wd_r : q1;
    assign read_data2 = byp2 ? wd_r : q2;

    // ---------------- a0 mirror for the LED ----------------
    reg [31:0] x10 = 32'd0;
    always @(posedge clk) begin
        if (rst) x10 <= 32'd0;
        else if (write_enable && write_reg == 5'd10) x10 <= write_data;
    end
    assign dbg_x10 = x10;

    // synthesis translate_off
    always @(posedge clk)
        if (!rst && write_enable && !read_en)
            $display("regfile_riscv WARNING: write_enable while read_en = 0 (contract violated) at %0t", $time);
    // synthesis translate_on

endmodule