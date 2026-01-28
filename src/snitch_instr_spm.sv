// Copyright 2025 ETH Zurich and University of Bologna.
// Solderpad Hardware License, Version 0.51, see LICENSE for details.
// SPDX-License-Identifier: SHL-0.51
//
// Instruction Scratchpad Memory (SPM) with banking
// Provides banked SRAM accessible via direct fetch ports + AXI refill interface

`include "common_cells/registers.svh"

module snitch_instr_spm #(
  parameter int unsigned AddrWidth = 48,
  parameter int unsigned DataWidth = 256,
  parameter int unsigned IdWidth = 4,
  parameter int unsigned SpmSize = 65536,  // in bytes
  parameter int unsigned NumBanks = 3,
  parameter int unsigned NumFetchPorts = 3,  // Number of parallel fetch interfaces
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

  localparam int unsigned BytesPerWord = DataWidth / 8;
  localparam int unsigned WordsPerBank = SpmSize / NumBanks / BytesPerWord;
  localparam int unsigned BankAddrWidth = $clog2(WordsPerBank);
  localparam int unsigned BankSelWidth = $clog2(NumBanks);
  localparam int unsigned TotalPorts = NumFetchPorts + 1; // fetch ports + AXI

  typedef logic [DataWidth-1:0] mem_data_t;
  typedef logic [DataWidth/8-1:0] mem_strb_t;

  // AXI memory interface from axi_to_mem
  logic [AddrWidth-1:0] axi_mem_addr;
  logic axi_mem_req;
  logic axi_mem_we;
  mem_strb_t axi_mem_be;
  mem_data_t axi_mem_wdata;
  mem_data_t axi_mem_rdata;
  logic axi_mem_req_q;

  // Bank selection and addressing (sequential banking)
  logic [BankSelWidth-1:0] axi_bank_sel, axi_bank_sel_q;
  logic [BankAddrWidth-1:0] axi_word_addr;
  
  assign axi_word_addr = axi_mem_addr[BankAddrWidth+$clog2(BytesPerWord)-1:$clog2(BytesPerWord)];
  assign axi_bank_sel = axi_mem_addr[BankAddrWidth+BankSelWidth+$clog2(BytesPerWord)-1:BankAddrWidth+$clog2(BytesPerWord)];

  `FF(axi_mem_req_q, axi_mem_req, 1'b0, clk_i, rst_ni)
  `FF(axi_bank_sel_q, axi_bank_sel, '0, clk_i, rst_ni)

  // Fetch port decoding
  logic [NumFetchPorts-1:0][BankSelWidth-1:0] fetch_bank_sel, fetch_bank_sel_q;
  logic [NumFetchPorts-1:0][BankAddrWidth-1:0] fetch_word_addr;
  logic [NumFetchPorts-1:0] fetch_req_q;
  
  for (genvar i = 0; i < NumFetchPorts; i++) begin : gen_fetch_decode
    assign fetch_word_addr[i] = fetch_addr_i[i][BankAddrWidth+$clog2(BytesPerWord)-1:$clog2(BytesPerWord)];
    assign fetch_bank_sel[i] = fetch_addr_i[i][BankAddrWidth+BankSelWidth+$clog2(BytesPerWord)-1:BankAddrWidth+$clog2(BytesPerWord)];
    `FF(fetch_req_q[i], fetch_req_i[i], 1'b0, clk_i, rst_ni)
    `FF(fetch_bank_sel_q[i], fetch_bank_sel[i], '0, clk_i, rst_ni)
  end

  axi_to_mem #(
    .axi_req_t (axi_req_t),
    .axi_resp_t (axi_rsp_t),
    .AddrWidth (AddrWidth),
    .DataWidth (DataWidth),
    .IdWidth (IdWidth),
    .NumBanks (1)
  ) i_axi_to_mem (
    .clk_i (clk_i),
    .rst_ni (rst_ni),
    .busy_o (),
    .axi_req_i (axi_req_i),
    .axi_resp_o (axi_rsp_o),
    .mem_req_o (axi_mem_req),
    .mem_gnt_i (axi_mem_req),
    .mem_addr_o (axi_mem_addr),
    .mem_wdata_o (axi_mem_wdata),
    .mem_strb_o (axi_mem_be),
    .mem_atop_o (),
    .mem_we_o (axi_mem_we),
    .mem_rvalid_i (axi_mem_req_q),
    .mem_rdata_i (axi_mem_rdata)
  );

  // Bank outputs for all ports
  mem_data_t [NumBanks-1:0][NumFetchPorts-1:0] fetch_bank_rdata;
  mem_data_t [NumBanks-1:0] axi_bank_rdata;

  // Multiplex read data from selected bank for each fetch port
  for (genvar i = 0; i < NumFetchPorts; i++) begin : gen_fetch_mux
    assign fetch_rdata_o[i] = fetch_bank_rdata[fetch_bank_sel_q[i]][i];
    assign fetch_rvalid_o[i] = fetch_req_q[i];
  end
  
  // Multiplex read data for AXI port
  assign axi_mem_rdata = axi_bank_rdata[axi_bank_sel_q];

  // Instantiate banked SRAM with per-bank arbitration
  for (genvar b = 0; b < NumBanks; b++) begin : gen_bank
    // Bank arbitration: AXI has priority, then round-robin among fetch ports
    logic bank_axi_req;
    logic [NumFetchPorts-1:0] bank_fetch_req;
    logic bank_we;
    logic [BankAddrWidth-1:0] bank_addr;
    mem_data_t bank_wdata;
    mem_strb_t bank_be;
    mem_data_t bank_rdata_out;
    
    // Check which ports want this bank
    assign bank_axi_req = axi_mem_req & (axi_bank_sel == b);
    for (genvar i = 0; i < NumFetchPorts; i++) begin : gen_check_fetch
      assign bank_fetch_req[i] = fetch_req_i[i] & (fetch_bank_sel[i] == b);
    end
    
    // Priority: AXI (for refill) > fetch port 0 > fetch port 1 > ...
    // AXI is write, fetch ports are read-only
    always_comb begin
      if (bank_axi_req) begin
        bank_we = axi_mem_we;
        bank_addr = axi_word_addr;
        bank_wdata = axi_mem_wdata;
        bank_be = axi_mem_be;
      end else begin
        // Find first requesting fetch port (priority encoder)
        bank_we = 1'b0;
        bank_addr = '0;
        bank_wdata = '0;
        bank_be = '0;
        for (int i = NumFetchPorts-1; i >= 0; i--) begin
          if (bank_fetch_req[i]) begin
            bank_addr = fetch_word_addr[i];
          end
        end
      end
    end
    
    tc_sram #(
      .NumWords (WordsPerBank),
      .DataWidth (DataWidth),
      .NumPorts (1)
    ) i_sram_bank (
      .clk_i (clk_i),
      .rst_ni (rst_ni),
      .req_i (bank_axi_req | (|bank_fetch_req)),
      .we_i (bank_we),
      .addr_i (bank_addr),
      .wdata_i (bank_wdata),
      .be_i (bank_be),
      .rdata_o (bank_rdata_out)
    );
    
    // Broadcast read data to all fetch ports (they select via mux)
    for (genvar i = 0; i < NumFetchPorts; i++) begin : gen_connect_fetch_rdata
      assign fetch_bank_rdata[b][i] = bank_rdata_out;
    end
    assign axi_bank_rdata[b] = bank_rdata_out;
  end

endmodule
