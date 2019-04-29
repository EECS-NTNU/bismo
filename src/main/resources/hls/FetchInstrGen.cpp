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
  size_t ETF_S,
  // capacity of LHS and RHS memories (in elements)
  size_t LMEM, size_t RMEM
>
void FetchInstrGen_RHSTiling_Templated(
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

  // mems are divided into regions to provide fetch-exec concurrency
  const uint8_t rmem_num_regions = (1 << ins_in.nbufs_res);
  const uint16_t rmem_region_size = (RMEM >> ins_in.nbufs_res);
  uint8_t rmem_region = 0;
  uint16_t rmem_region_offset = 0;

  for(uint16_t n = 0; n < ins_in.tiles_n; n++) {
    // start by acquiring buffer to fill
    // receive token from execute stage
    sync.isSendToken = 0;
    sync.chanID = 0;
    out.write(sync.asRaw());
    ap_wait();

    // do not fetch LHS; assume already fetched
    // create fetch instruction for all bit positions of
    // current RHS slice
    const int first_lhs_id = 0;
    const int first_rhs_id = M;
    const int bytes_per_rhs_tile = (N * K) / 8;

    // each bit position is one block
    fetch.dram_block_count = ins_in.bits_r;
    // each block is a group of Dn rows' worth of bits
    fetch.dram_block_size_bytes = ins_in.tiles_k * bytes_per_rhs_tile;
    // block stride/skip is one bit position worth of bits
    fetch.dram_block_offset_bytes = ins_in.tiles_n * ins_in.tiles_k * bytes_per_rhs_tile;
    // IMPORTANT TODO: put in SW assertions around sizes of these, especially
    // dram_block_offset_bytes! other option is to generate one
    // fetch instruction per bit position...
    // DRAM base address for LHS
    fetch.dram_base = ins_in.dram_rhs + n * ins_in.tiles_k * bytes_per_rhs_tile;


    fetch.bram_addr_base = (ins_in.base_r + rmem_region_offset) << ETF_S;
    fetch.bram_id_start = first_rhs_id;
    fetch.bram_id_range = 1;
    // how many DRAM data words are copied before the
    // fetch interconnect starts targeting the next BRAM
    fetch.tiles_per_row = ins_in.tiles_k << ETF_S;

    // emit fetch instruction for RHS matrix
    out.write(fetch.asRaw());
    ap_wait();

    // signal that buffer is now filled
    // send token to execute stage
    sync.isSendToken = 1;
    sync.chanID = 0;
    out.write(sync.asRaw());

    // use the next rmem region for following fetch
    rmem_region++;
    rmem_region_offset += rmem_region_size;
    if(rmem_region == rmem_num_regions) {
      rmem_region = 0;
      rmem_region_offset = 0;
    }
  }
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

  FetchInstrGen_RHSTiling_Templated<
    TEMPLATE_PARAM_M, TEMPLATE_PARAM_K, TEMPLATE_PARAM_N,
    TEMPLATE_PARAM_ETF_S, TEMPLATE_PARAM_LMEM, TEMPLATE_PARAM_RMEM
  >(
    in, out
  );
}
