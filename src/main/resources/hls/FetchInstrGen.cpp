/*******************************************************************************
# Copyright (c) 2018, Xilinx, Inc.
# All rights reserved.
# Author: Yaman Umuroglu
#
# Redistribution and use in source and binary forms, with or without modification,
# are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice,
# this list of conditions and the following disclaimer.
#
#
# 2. Redistributions in binary form must reproduce the above copyright notice,
# this list of conditions and the following disclaimer in the documentation
# and/or other materials provided with the distribution.
#
#
# 3. Neither the name of the copyright holder nor the names of its contributors
# may be used to endorse or promote products derived from this software
# without specific prior written permission.
#
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
# IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
# INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
# DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY
# OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
# NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
# EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#
# *******************************************************************************/

#include <ap_int.h>
#include <ap_utils.h>
#include <hls_stream.h>
#include <stdint.h>
#include "BISMOInstruction.hpp"

template <
  // matmul array dimensions: rows, common, cols
  size_t M, size_t K, size_t N,
  // exec-to-fetch left-shift ratio: log2(K / fetch width)
  size_t ETF_S
>
void FetchInstrGen_Templated(
  hls::stream<ap_uint<BISMO_MMDESCR_BITS>> & in,
  hls::stream<ap_uint<BISMO_INSTR_BITS>> & out
) {
  #pragma HLS INTERFACE ap_ctrl_none port=return
  #pragma HLS INTERFACE axis port=out
  #pragma HLS INTERFACE axis port=in
io_section:{
  #pragma HLS protocol fixed

  BISMOFetchRunInstruction fetch;
  BISMOSyncInstruction sync;

  // set the invariants (values do not depend on loop iter)
  sync.targetStage = stgFetch;
  fetch.targetStage = stgFetch;
  fetch.isRunCfg = 1;
  sync.isRunCfg = 0;
  // read the descriptor
  SingleMMDescriptor ins_in;
  ins_in.fromRaw(in.read());
  ap_wait();

  // start by acquiring buffer to fill
  // receive token from execute stage
  sync.isSendToken = 0;
  sync.chanID = 0;
  out.write(sync.asRaw());
  ap_wait();

  // number of lhs and rhs tiles
  const uint32_t lhs_tiles = ins_in.tiles_m * ins_in.tiles_k;
  const uint32_t rhs_tiles = ins_in.tiles_n * ins_in.tiles_k;

  const int first_lhs_id = 0;
  const int first_rhs_id = M;
  const int bytes_per_lhs_tile = (M * K) / 8;
  const int bytes_per_rhs_tile = (N * K) / 8;

  // do not fetch LHS if addr is 0xFFFFFFFF (already fetched)
  if(ins_in.dram_lhs != 0xFFFFFFFF) {
    // prepare fetch instruction for LHS matrix
    fetch.bram_addr_base = ins_in.base_l << ETF_S;
    fetch.bram_id_start = first_lhs_id;
    fetch.bram_id_range = M - 1;
    // how many DRAM data words are copied before the
    // fetch interconnect starts targeting the next BRAM
    fetch.tiles_per_row = ins_in.tiles_k << ETF_S;
    // DRAM base address for LHS
    fetch.dram_base = ins_in.dram_lhs;
    // bytes to read in each contiguous block
    fetch.dram_block_size_bytes = lhs_tiles * ins_in.bits_l * bytes_per_lhs_tile;
    // TODO partial matrices will need multiple blocks here
    fetch.dram_block_count = 1;
    fetch.dram_block_offset_bytes = 0;
    // emit fetch instruction for LHS matrix
    out.write(fetch.asRaw());
    ap_wait();
  }

  // prepare fetch instruction for RHS matrix
  fetch.bram_addr_base = ins_in.base_r << ETF_S;
  fetch.bram_id_start = first_rhs_id;
  fetch.bram_id_range = N - 1;
  // how many DRAM data words are copied before the
  // fetch interconnect starts targeting the next BRAM
  fetch.tiles_per_row = ins_in.tiles_k << ETF_S;
  // DRAM base address for LHS
  fetch.dram_base = ins_in.dram_rhs;
  // bytes to read in each contiguous block
  fetch.dram_block_size_bytes = rhs_tiles * ins_in.bits_r * bytes_per_rhs_tile;
  // TODO partial matrices will need multiple blocks here
  fetch.dram_block_count = 1;
  fetch.dram_block_offset_bytes = 0;
  // emit fetch instruction for RHS matrix
  out.write(fetch.asRaw());
  ap_wait();

  // signal that buffer is now filled
  // send token to execute stage
  sync.isSendToken = 1;
  sync.chanID = 0;
  out.write(sync.asRaw());
}
}

#include "FetchInstrGen_TemplateDefs.hpp"
void FetchInstrGen(
  hls::stream<ap_uint<BISMO_MMDESCR_BITS>> & in,
  hls::stream<ap_uint<BISMO_INSTR_BITS>> & out
) {
  #pragma HLS INTERFACE ap_ctrl_none port=return
  #pragma HLS INTERFACE axis port=out
  #pragma HLS INTERFACE axis port=in

  FetchInstrGen_Templated<
    TEMPLATE_PARAM_M, TEMPLATE_PARAM_K, TEMPLATE_PARAM_N,
    TEMPLATE_PARAM_ETF_S
  >(
    in, out
  );
}
