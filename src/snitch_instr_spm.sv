// Copyright 2025 ETH Zurich and University of Bologna.
// Solderpad Hardware License, Version 0.51, see LICENSE for details.
// SPDX-License-Identifier: SHL-0.51
//
// Instruction Scratchpad Memory (SPM) with banking
// - Banked SRAM, one single-port SRAM per bank
// - Multiple direct fetch ports (variable-latency via rvalid)
// - AXI slave refill via axi_to_mem
//
// FIXES vs previous version:
// - SRAM addr_i is now driven by the *selected requester* (AXI or a fetch port),
//   not hardwired to AXI decode.
// - No bogus "xbar" that only routes bank-id; we arbitrate per bank and forward
//   (req,we,addr,wdata,be) properly.
// - AXI-to-mem handshake uses mem_gnt_i (not hardwired 1).
// - Fetch ports are buffered (1 entry per port) so bank conflicts / refill conflicts
//   stall cleanly (rvalid delayed) instead of returning wrong data.

`include "common_cells/registers.svh"

module snitch_instr_spm #(
  parameter int unsigned AddrWidth     = 48,
  parameter int unsigned DataWidth     = 256,
  parameter int unsigned IdWidth       = 4,
  parameter int unsigned SpmSize       = 65536,  // bytes
  parameter logic [AddrWidth-1:0] SpmBaseAddr = 0,
  parameter int unsigned NumBanks      = 4,
  parameter int unsigned NumFetchPorts = 3,
  parameter type axi_req_t = logic,
  parameter type axi_rsp_t = logic
) (
  input  logic clk_i,
  input  logic rst_ni,

  // Direct fetch ports (from L0 prefetchers)
  input  logic [NumFetchPorts-1:0]                     fetch_req_i,
  input  logic [NumFetchPorts-1:0][AddrWidth-1:0]      fetch_addr_i,
  output logic [NumFetchPorts-1:0][DataWidth-1:0]      fetch_rdata_o,
  output logic [NumFetchPorts-1:0]                     fetch_rvalid_o,

  // AXI slave for refill
  input  axi_req_t axi_req_i,
  output axi_rsp_t axi_rsp_o
);

  // ------------------------
  // Parameters / localparams
  // ------------------------
  localparam int unsigned BytesPerWord   = DataWidth / 8;
  localparam int unsigned ByteOffWidth   = (BytesPerWord <= 1) ? 1 : $clog2(BytesPerWord);

  localparam int unsigned WordsPerBank   = SpmSize / NumBanks / BytesPerWord;

  localparam int unsigned BankAddrWidth  = (WordsPerBank <= 1) ? 1 : $clog2(WordsPerBank);
  localparam int unsigned BankSelWidth   = (NumBanks    <= 1) ? 1 : $clog2(NumBanks);
  localparam int unsigned FetchIdxWidth  = (NumFetchPorts <= 1) ? 1 : $clog2(NumFetchPorts);

  typedef logic [DataWidth-1:0]     mem_data_t;
  typedef logic [DataWidth/8-1:0]   mem_strb_t;

  localparam logic [AddrWidth-1:0] SpmSizeBytes = AddrWidth'(SpmSize);
  localparam logic [AddrWidth-1:0] SpmEndAddr   = SpmBaseAddr + SpmSizeBytes;

`ifndef SYNTHESIS
  initial begin
    if ((DataWidth % 8) != 0) $fatal(1, "DataWidth must be multiple of 8");
    if ((SpmSize % (NumBanks * BytesPerWord)) != 0) $fatal(1, "SpmSize must be divisible by NumBanks*BytesPerWord");
    if ((1 << $clog2(NumBanks)) != NumBanks) $fatal(1, "NumBanks must be power of two for this address decode");
  end
`endif

  // ------------------------
  // Fetch request buffering (1-entry per port)
  // ------------------------
  logic [NumFetchPorts-1:0]                  fetch_pending_q;
  logic [NumFetchPorts-1:0][BankSelWidth-1:0] fetch_bank_q;
  logic [NumFetchPorts-1:0][BankAddrWidth-1:0] fetch_word_q;

  // Combinational decode for incoming fetch addresses
  logic [NumFetchPorts-1:0]                   fetch_in_range;
  logic [NumFetchPorts-1:0][AddrWidth-1:0]    fetch_off;
  logic [NumFetchPorts-1:0][BankSelWidth-1:0] fetch_bank_dec;
  logic [NumFetchPorts-1:0][BankAddrWidth-1:0] fetch_word_dec;

  for (genvar i = 0; i < NumFetchPorts; i++) begin : gen_fetch_decode
    assign fetch_in_range[i] = (fetch_addr_i[i] >= SpmBaseAddr) && (fetch_addr_i[i] < SpmEndAddr);
    assign fetch_off[i]      = fetch_addr_i[i] - SpmBaseAddr;

    // Contiguous banking: [ ... | bank_sel | bank_word | byte_off ]
    assign fetch_word_dec[i] = fetch_off[i][ByteOffWidth + BankAddrWidth - 1 : ByteOffWidth];
    assign fetch_bank_dec[i] = fetch_off[i][ByteOffWidth + BankAddrWidth + BankSelWidth - 1 : ByteOffWidth + BankAddrWidth];
  end

  // Effective fetch request set used for arbitration:
  // - if pending exists, use stored bank/word
  // - else, use current inputs (pulse)
  logic [NumFetchPorts-1:0]                   fetch_req_eff;
  logic [NumFetchPorts-1:0][BankSelWidth-1:0] fetch_bank_eff;
  logic [NumFetchPorts-1:0][BankAddrWidth-1:0] fetch_word_eff;

  always_comb begin
    for (int i = 0; i < NumFetchPorts; i++) begin
      if (fetch_pending_q[i]) begin
        fetch_req_eff[i]  = 1'b1;
        fetch_bank_eff[i] = fetch_bank_q[i];
        fetch_word_eff[i] = fetch_word_q[i];
      end else begin
        fetch_req_eff[i]  = fetch_req_i[i] & fetch_in_range[i];
        fetch_bank_eff[i] = fetch_bank_dec[i];
        fetch_word_eff[i] = fetch_word_dec[i];
      end
    end
  end

  // ------------------------
  // AXI to memory converter (single stream)
  // ------------------------
  logic        axi_mem_req;
  logic        axi_mem_gnt;
  logic        axi_mem_we;
  logic [AddrWidth-1:0] axi_mem_addr;
  mem_data_t   axi_mem_wdata;
  mem_strb_t   axi_mem_be;
  mem_data_t   axi_mem_rdata;
  logic        axi_mem_rvalid;

  axi_to_mem #(
    .axi_req_t   (axi_req_t),
    .axi_resp_t  (axi_rsp_t),
    .AddrWidth   (AddrWidth),
    .DataWidth   (DataWidth),
    .IdWidth     (IdWidth),
    .NumBanks    (1)
  ) i_axi_to_mem (
    .clk_i        (clk_i),
    .rst_ni       (rst_ni),
    .busy_o       (),
    .axi_req_i    (axi_req_i),
    .axi_resp_o   (axi_rsp_o),

    .mem_req_o    (axi_mem_req),
    .mem_gnt_i    (axi_mem_gnt),
    .mem_addr_o   (axi_mem_addr),
    .mem_wdata_o  (axi_mem_wdata),
    .mem_strb_o   (axi_mem_be),
    .mem_atop_o   (),
    .mem_we_o     (axi_mem_we),

    .mem_rvalid_i (axi_mem_rvalid),
    .mem_rdata_i  (axi_mem_rdata)
  );

  // Decode AXI target bank/word
  logic                  axi_in_range;
  logic [AddrWidth-1:0]   axi_off;
  logic [BankSelWidth-1:0] axi_bank_sel;
  logic [BankAddrWidth-1:0] axi_word_addr;

  assign axi_in_range  = (axi_mem_addr >= SpmBaseAddr) && (axi_mem_addr < SpmEndAddr);
  assign axi_off       = axi_mem_addr - SpmBaseAddr;
  assign axi_word_addr = axi_off[ByteOffWidth + BankAddrWidth - 1 : ByteOffWidth];
  assign axi_bank_sel  = axi_off[ByteOffWidth + BankAddrWidth + BankSelWidth - 1 : ByteOffWidth + BankAddrWidth];

  // ------------------------
  // Per-bank arbitration + SRAM drive
  // ------------------------
  logic [NumBanks-1:0]                 bank_req;
  logic [NumBanks-1:0]                 bank_we;
  logic [NumBanks-1:0][BankAddrWidth-1:0] bank_addr;
  mem_data_t [NumBanks-1:0]            bank_wdata;
  mem_strb_t [NumBanks-1:0]            bank_be;
  mem_data_t [NumBanks-1:0]            bank_rdata;

  // RR pointer among fetch ports per bank
  logic [NumBanks-1:0][FetchIdxWidth-1:0] rr_ptr_q;

  // Grant bookkeeping (current cycle)
  logic [NumBanks-1:0] bank_grant_axi;
  logic [NumBanks-1:0] bank_grant_fetch_valid;
  logic [NumBanks-1:0][FetchIdxWidth-1:0] bank_grant_fetch_idx;

  logic [NumFetchPorts-1:0]                 fetch_grant;      // request accepted into SRAM this cycle
  logic [NumFetchPorts-1:0][BankSelWidth-1:0] fetch_grant_bank;

  always_comb begin
    // Declare all variables first
    logic axi_targets_this_bank;
    logic [NumFetchPorts-1:0] req_vec;
    int unsigned start;
    int sel;

    // defaults
    for (int b = 0; b < NumBanks; b++) begin
      bank_grant_axi[b]         = 1'b0;
      bank_grant_fetch_valid[b] = 1'b0;
      bank_grant_fetch_idx[b]   = '0;

      bank_req[b]   = 1'b0;
      bank_we[b]    = 1'b0;
      bank_addr[b]  = '0;
      bank_wdata[b] = '0;
      bank_be[b]    = '0;
    end

    for (int i = 0; i < NumFetchPorts; i++) begin
      fetch_grant[i]      = 1'b0;
      fetch_grant_bank[i] = '0;
    end

    // Per bank: AXI has priority when targeting that bank; otherwise serve one fetch via RR
    for (int b = 0; b < NumBanks; b++) begin
      axi_targets_this_bank = axi_mem_req && axi_in_range && (axi_bank_sel == BankSelWidth'(b));

      if (axi_targets_this_bank) begin
        // Grant AXI on this bank
        bank_grant_axi[b] = 1'b1;

        bank_req[b]   = 1'b1;
        bank_we[b]    = axi_mem_we;
        bank_addr[b]  = axi_word_addr;
        bank_wdata[b] = axi_mem_wdata;
        bank_be[b]    = axi_mem_be;
      end else begin
        // Build request vector for fetch ports targeting this bank
        req_vec = '0;
        for (int i = 0; i < NumFetchPorts; i++) begin
          req_vec[i] = fetch_req_eff[i] && (fetch_bank_eff[i] == BankSelWidth'(b));
        end

        // RR select
        start = rr_ptr_q[b];
        sel = -1;
        for (int k = 0; k < NumFetchPorts; k++) begin
          int idx;
          idx = start + k;
          if (idx >= NumFetchPorts) idx -= NumFetchPorts;
          if ((sel == -1) && req_vec[idx]) sel = idx;
        end

        if (sel != -1) begin
          bank_grant_fetch_valid[b] = 1'b1;
          bank_grant_fetch_idx[b]   = FetchIdxWidth'(sel);

          // Drive SRAM read
          bank_req[b]  = 1'b1;
          bank_we[b]   = 1'b0;
          bank_addr[b] = fetch_word_eff[sel];

          // Mark fetch grant for response routing
          fetch_grant[sel]      = 1'b1;
          fetch_grant_bank[sel] = BankSelWidth'(b);
        end
      end
    end
  end

  // AXI memory grant: true if we granted AXI on its target bank this cycle
  always_comb begin
    axi_mem_gnt = 1'b0;
    if (axi_mem_req && axi_in_range) begin
      for (int b = 0; b < NumBanks; b++) begin
        if (bank_grant_axi[b]) axi_mem_gnt = 1'b1;
      end
    end
  end

  // ------------------------
  // SRAM banks
  // ------------------------
  for (genvar b = 0; b < NumBanks; b++) begin : gen_bank
    tc_sram #(
      .NumWords  (WordsPerBank),
      .DataWidth (DataWidth),
      .NumPorts  (1)
    ) i_sram_bank (
      .clk_i   (clk_i),
      .rst_ni  (rst_ni),
      .req_i   (bank_req[b]),
      .we_i    (bank_we[b]),
      .addr_i  (bank_addr[b]),
      .wdata_i (bank_wdata[b]),
      .be_i    (bank_be[b]),
      .rdata_o (bank_rdata[b])
    );
  end

  // ------------------------
  // Response pipelining (1-cycle SRAM read latency)
  // ------------------------
  // Fetch responses
  logic [NumFetchPorts-1:0]                   fetch_resp_v_q;
  logic [NumFetchPorts-1:0][BankSelWidth-1:0] fetch_resp_bank_q;

  // AXI read response
  logic                   axi_read_resp_v_q;
  logic [BankSelWidth-1:0] axi_read_resp_bank_q;

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      fetch_pending_q         <= '0;
      fetch_bank_q            <= '0;
      fetch_word_q            <= '0;
      fetch_resp_v_q          <= '0;
      fetch_resp_bank_q       <= '0;

      axi_read_resp_v_q       <= 1'b0;
      axi_read_resp_bank_q    <= '0;

      rr_ptr_q                <= '0;
    end else begin
      // Default: response valids clear unless set below (1-cycle pulse)
      fetch_resp_v_q    <= '0;
      axi_read_resp_v_q <= 1'b0;

      // Capture new fetch pulses into pending buffer if not immediately granted this cycle
      for (int i = 0; i < NumFetchPorts; i++) begin
        logic capture;
        capture = (fetch_req_i[i] && fetch_in_range[i] && !fetch_pending_q[i] && !fetch_grant[i]);

`ifndef SYNTHESIS
        if (fetch_req_i[i] && fetch_in_range[i] && fetch_pending_q[i]) begin
          // If this triggers, your prefetcher is issuing a new request while one is still pending.
          // Increase buffering or enforce single-outstanding per port upstream.
          $warning("snitch_instr_spm: fetch port %0d request while pending; new request ignored", i);
        end
`endif

        if (capture) begin
          fetch_pending_q[i] <= 1'b1;
          fetch_bank_q[i]    <= fetch_bank_dec[i];
          fetch_word_q[i]    <= fetch_word_dec[i];
        end

        // If granted (served), clear pending and arm response for next cycle
        if (fetch_grant[i]) begin
          fetch_pending_q[i]   <= 1'b0;
          fetch_resp_v_q[i]    <= 1'b1;
          fetch_resp_bank_q[i] <= fetch_grant_bank[i];
        end
      end

      // Update RR pointers when a fetch grant happens on that bank
      for (int b = 0; b < NumBanks; b++) begin
        if (bank_grant_fetch_valid[b]) begin
          int unsigned next;
          next = bank_grant_fetch_idx[b] + 1;
          if (next >= NumFetchPorts) next = 0;
          rr_ptr_q[b] <= FetchIdxWidth'(next);
        end
      end

      // AXI read response valid (only for reads), 1 cycle after grant
      if (axi_mem_gnt && !axi_mem_we && axi_mem_req && axi_in_range) begin
        axi_read_resp_v_q    <= 1'b1;
        axi_read_resp_bank_q <= axi_bank_sel;
      end
    end
  end

  // Drive fetch outputs
  for (genvar i = 0; i < NumFetchPorts; i++) begin : gen_fetch_out
    assign fetch_rvalid_o[i] = fetch_resp_v_q[i];
    assign fetch_rdata_o[i]  = bank_rdata[fetch_resp_bank_q[i]];
  end

  // Drive AXI-to-mem read response back
  assign axi_mem_rvalid = axi_read_resp_v_q;
  assign axi_mem_rdata  = bank_rdata[axi_read_resp_bank_q];

endmodule
