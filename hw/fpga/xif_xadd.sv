
// Minimal CV-X-IF coprocessor for X-HEEP / CV32E40PX
// Implements a single custom instruction:
//   XADD rd, rs1, rs2  (opcode=0x0B custom-0, funct3=0, funct7=0x01)
// Result: rd = rs1 + rs2
//
// This module connects to the if_xif interface (CORE-V X-IF / CV-X-IF).

module xif_xadd (
    input logic clk_i,
    input logic rst_ni,

    // CV-X-IF channels (modports)
    if_xif.coproc_compressed xif_compressed_if,
    if_xif.coproc_issue      xif_issue_if,
    if_xif.coproc_commit     xif_commit_if,
    if_xif.coproc_mem        xif_mem_if,
    if_xif.coproc_result     xif_result_if
);

  // Handy local widths derived from the interface instance
localparam int unsigned ID_W   = 4;
localparam int unsigned DATA_W = 32;

  logic               pending_q;
  logic [ID_W   -1:0] id_q;
  logic [        4:0] rd_q;
  logic [ DATA_W-1:0] data_q;

  // logic [       31:0] instr;
  logic [6:0] opcode;
  logic [2:0] funct3;
  logic [6:0] funct7;
  logic [4:0] rd;
  logic               is_xadd;
  logic               hs_issue;
  logic               hs_result;

  assign opcode = xif_issue_if.issue_req.instr[6:0];
  assign funct3 = xif_issue_if.issue_req.instr[14:12];
  assign funct7 = xif_issue_if.issue_req.instr[31:25];
  assign rd     = xif_issue_if.issue_req.instr[11:7];

  // // Decode the offloaded instruction
  // assign instr = xif_issue_if.issue_req.instr;

  // opcode[6:0]   = 0x0B (custom-0)
  // funct3[14:12] = 0
  // funct7[31:25] = 0x01
  //assign is_xadd = (instr[6:0] == 7'h0B) && (instr[14:12] == 3'b000) && (instr[31:25] == 7'h01);
  assign is_xadd = (opcode == 7'h0B) && (funct3 == 3'b000) && (funct7 == 7'h01);

  // Handshakes
  assign hs_issue = xif_issue_if.issue_valid && xif_issue_if.issue_ready && (is_xadd && !pending_q);

  assign hs_result = xif_result_if.result_valid && xif_result_if.result_ready;

  // ----------------------------------------------------------------------------
  // Issue channel: accept/describe the offloaded instruction
  // ----------------------------------------------------------------------------
  always_comb begin
    // Default outputs
    xif_issue_if.issue_resp           = '0;

    // For unsupported instructions, we keep accept=0 (CPU will treat as illegal).
    // For XADD, we accept only if no pending result.
    xif_issue_if.issue_resp.accept    = (is_xadd && !pending_q);
    xif_issue_if.issue_resp.writeback = (is_xadd && !pending_q);
    xif_issue_if.issue_resp.dualwrite = 1'b0;
    xif_issue_if.issue_resp.dualread  = 3'b000;
    xif_issue_if.issue_resp.loadstore = 1'b0;
    xif_issue_if.issue_resp.ecswrite  = 1'b0;
    xif_issue_if.issue_resp.exc       = 1'b0;

    // Ready behavior:
    // - If this is our instruction and we're busy, stall.
    // - Otherwise, be ready so the core can observe accept=0 quickly.
    xif_issue_if.issue_ready          = (!pending_q) || (!is_xadd);
  end

  // ----------------------------------------------------------------------------
  // Result channel: send the writeback back to the core
  // ----------------------------------------------------------------------------
  always_comb begin
    xif_result_if.result_valid = pending_q;
    xif_result_if.result       = '0;

    if (pending_q) begin
      xif_result_if.result.id      = id_q;
      xif_result_if.result.data    = data_q;
      xif_result_if.result.rd      = rd_q;

      // Single writeback (one XLEN word). For XLEN=32 and X_RFW_WIDTH=32, this is 1 bit.
      xif_result_if.result.we      = '1;

      // No ECS writes / no exceptions / no errors
      xif_result_if.result.ecsdata = '0;
      xif_result_if.result.ecswe   = '0;
      xif_result_if.result.exc     = 1'b0;
      xif_result_if.result.exccode = '0;
      xif_result_if.result.err     = 1'b0;
      xif_result_if.result.dbg     = 1'b0;
    end
  end

  // ----------------------------------------------------------------------------
  // Compressed channel: not supported -> always reject
  // ----------------------------------------------------------------------------
  always_comb begin
    xif_compressed_if.compressed_ready = 1'b1;
    xif_compressed_if.compressed_resp = '0;
    xif_compressed_if.compressed_resp.accept = 1'b0;
    // Provide a harmless instruction (NOP) in case it's ever sampled
    xif_compressed_if.compressed_resp.instr = 32'h0000_0013;  // addi x0,x0,0
  end

  // ----------------------------------------------------------------------------
  // Memory channel: not used -> always idle
  // ----------------------------------------------------------------------------
  always_comb begin
    xif_mem_if.mem_valid = 1'b0;
    xif_mem_if.mem_req   = '0;
  end

  // ----------------------------------------------------------------------------
  // State update: latch accepted instruction & clear when result is consumed / killed
  // ----------------------------------------------------------------------------
  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      pending_q <= 1'b0;
      id_q      <= '0;
      rd_q      <= '0;
      data_q    <= '0;
    end else begin
      // Kill notification from the core (optional)
      if (pending_q &&
          xif_commit_if.commit_valid &&
          xif_commit_if.commit.commit_kill &&
          (xif_commit_if.commit.id == id_q)) begin
        pending_q <= 1'b0;
      end  // Result handshake: core accepted the result
      else if (hs_result) begin
        pending_q <= 1'b0;
      end  // New accepted instruction
      else if (hs_issue) begin
        pending_q <= 1'b1;
        id_q      <= xif_issue_if.issue_req.id;
        rd_q      <= rd;
        data_q    <= xif_issue_if.issue_req.rs[0] + xif_issue_if.issue_req.rs[1];
      end
    end
  end

endmodule
