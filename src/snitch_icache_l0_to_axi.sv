// Copyright 2025 ETH Zurich and University of Bologna.
// Solderpad Hardware License, Version 0.51, see LICENSE for details.
// SPDX-License-Identifier: SHL-0.51
//
// L0 Prefetcher to AXI Adapter (SPM Mode)
// Directly converts L0 prefetch requests to AXI burst reads
// Bypasses L1 cache completely when instruction SPM is present

`include "common_cells/registers.svh"

module snitch_icache_l0_to_axi import snitch_icache_pkg::*; #(
  parameter snitch_icache_pkg::config_t CFG = '0,
  parameter type axi_req_t = logic,
  parameter type axi_rsp_t = logic
) (
  input  logic clk_i,
  input  logic rst_ni,

  // L0 prefetcher interface
  input  logic [CFG.FETCH_AW-1:0] in_addr_i,
  input  logic [CFG.ID_WIDTH-1:0] in_id_i,
  input  logic                    in_valid_i,
  output logic                    in_ready_o,

  output logic [CFG.ID_WIDTH-1:0]   out_id_o,
  output logic [CFG.LINE_WIDTH-1:0] out_data_o,
  output logic                      out_error_o,
  output logic                      out_valid_o,
  input  logic                      out_ready_i,

  // AXI master interface
  output axi_req_t axi_req_o,
  input  axi_rsp_t axi_rsp_i
);

  localparam int unsigned AxiDataWidth = CFG.FILL_DW;
  localparam int unsigned BeatsPerLine = CFG.LINE_WIDTH / AxiDataWidth;
  localparam int unsigned BeatCntWidth = BeatsPerLine > 1 ? $clog2(BeatsPerLine) : 1;

  // Align address to cache line boundary
  logic [CFG.FETCH_AW-1:0] addr_aligned;
  assign addr_aligned = {in_addr_i[CFG.FETCH_AW-1:CFG.LINE_ALIGN], {CFG.LINE_ALIGN{1'b0}}};

  // State machine
  typedef enum logic [1:0] {
    IDLE,
    WAIT_AR,
    WAIT_R,
    PRESENT
  } state_e;

  state_e state_d, state_q;

  // Transaction tracking
  logic [CFG.ID_WIDTH-1:0] trans_id_d, trans_id_q;
  logic [CFG.LINE_WIDTH-1:0] trans_data_d, trans_data_q;
  logic trans_error_d, trans_error_q;
  logic [BeatCntWidth-1:0] beat_cnt_d, beat_cnt_q;

  // State machine logic
  always_comb begin
    state_d = state_q;
    trans_id_d = trans_id_q;
    trans_data_d = trans_data_q;
    trans_error_d = trans_error_q;
    beat_cnt_d = beat_cnt_q;

    in_ready_o = 1'b0;
    out_valid_o = 1'b0;
    out_id_o = trans_id_q;
    out_data_o = trans_data_q;
    out_error_o = trans_error_q;

    axi_req_o = '0;

    case (state_q)
      IDLE: begin
        if (in_valid_i) begin
          // Latch request
          trans_id_d = in_id_i;
          trans_data_d = '0;
          trans_error_d = 1'b0;
          beat_cnt_d = '0;
          state_d = WAIT_AR;
        end
      end

      WAIT_AR: begin
        // Issue AXI AR (read address) transaction
        axi_req_o.ar_valid = 1'b1;
        axi_req_o.ar.addr = addr_aligned;
        axi_req_o.ar.len = BeatsPerLine - 1; // AXI len = bursts - 1
        axi_req_o.ar.size = $clog2(AxiDataWidth/8); // Bytes per beat
        axi_req_o.ar.burst = 2'b01; // INCR burst
        axi_req_o.ar.lock = 1'b0;
        axi_req_o.ar.cache = 4'b0010; // Normal non-cacheable bufferable
        axi_req_o.ar.prot = 3'b100; // Instruction, non-secure, unprivileged
        axi_req_o.ar.qos = 4'b0000;
        axi_req_o.ar.region = 4'b0000;
        axi_req_o.ar.id = '0;
        axi_req_o.ar.user = '0;

        if (axi_rsp_i.ar_ready) begin
          state_d = WAIT_R;
        end
      end

      WAIT_R: begin
        // Receive AXI R (read data) beats
        axi_req_o.r_ready = 1'b1;

        if (axi_rsp_i.r_valid) begin
          // Accumulate data
          trans_data_d[(beat_cnt_q * AxiDataWidth) +: AxiDataWidth] = axi_rsp_i.r.data;
          trans_error_d = trans_error_q | (axi_rsp_i.r.resp != 2'b00);
          beat_cnt_d = beat_cnt_q + 1;

          if (axi_rsp_i.r.last) begin
            state_d = PRESENT;
          end
        end
      end

      PRESENT: begin
        out_valid_o = 1'b1;

        if (out_ready_i) begin
          in_ready_o = 1'b1; // Acknowledge original request
          state_d = IDLE;
        end
      end

      default: state_d = IDLE;
    endcase
  end

  // Registers
  `FF(state_q, state_d, IDLE)
  `FF(trans_id_q, trans_id_d, '0)
  `FF(trans_data_q, trans_data_d, '0)
  `FF(trans_error_q, trans_error_d, '0)
  `FF(beat_cnt_q, beat_cnt_d, '0)

endmodule
