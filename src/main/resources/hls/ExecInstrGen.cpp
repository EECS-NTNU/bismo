// Copyright (c) 2019 Xilinx
//
// BSD v3 License
//
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// * Redistributions of source code must retain the above copyright notice, this
//   list of conditions and the following disclaimer.
//
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
//
// * Neither the name of BISMO nor the names of its
//   contributors may be used to endorse or promote products derived from
//   this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#include <ap_int.h>
#include <ap_utils.h>
#include <hls_stream.h>
#include <stdint.h>
#include "BISMOInstruction.hpp"

template <
  // capacity of LHS and RHS memories (in elements)
  size_t LMEM, size_t RMEM,
  // exec-to-fetch left-shift ratio: log2(K / fetch width)
  size_t ETF_S
>
void ExecInstrGen_RHSLHSTiling(
  hls::stream<ap_uint<BISMO_MMDESCR_BITS>> & in,
  hls::stream<ap_uint<BISMO_INSTR_BITS>> & out
) {
  #pragma HLS INTERFACE ap_ctrl_none port=return
  #pragma HLS INTERFACE axis port=out
  #pragma HLS INTERFACE axis port=in
io_section:{
  #pragma HLS protocol fixed

  BISMOExecRunInstruction exec;
  BISMOSyncInstruction sync;

  // read the descriptor
  SingleMMDescriptor ins_in;
  ins_in.fromRaw(in.read());
  ap_wait();
  sync.targetStage = stgExec;
  exec.targetStage = stgExec;
  exec.isRunCfg = 1;
  sync.isRunCfg = 0;

  // compute the size of the iteration space
  const size_t total_iters = ins_in.tiles_m * ins_in.tiles_n * ins_in.bits_l * ins_in.bits_r;
  /// iteration variables
  uint16_t m = 0, n = 0;
  uint8_t slice = 0;
  uint8_t z1 = slice < ins_in.bits_r ? 0 : (slice - ins_in.bits_r + 1);
  uint8_t z2 = slice < ins_in.bits_l ? 0 : (slice - ins_in.bits_l + 1);
  int8_t j = slice - z2;
  uint8_t offset_res = 0;
  // l and r are derived from the loop indices
  uint8_t l = 0, r = 0;
  // mems are divided into regions to provide fetch-exec concurrency
  const uint8_t lmem_num_regions = (1 << ins_in.nbufs_fetch_exec_log2);
  const uint16_t lmem_region_size = (LMEM >> ins_in.nbufs_fetch_exec_log2);
  uint8_t lmem_region = 0;
  uint16_t lmem_region_offset = 0;
  const uint8_t rmem_num_regions = (1 << ins_in.nbufs_fetch_exec_log2);
  const uint16_t rmem_region_size = (RMEM >> ins_in.nbufs_fetch_exec_log2);
  uint8_t rmem_region = 0;
  uint16_t rmem_region_offset = 0;
  // single iteration space for the entire instrgen
  for(size_t i = 0; i < total_iters; i++) {
    #pragma HLS PIPELINE II=1
    // compute l and r from loop indices
    const int l = ins_in.bits_l - j - 1;
    const int r = ins_in.bits_r - (slice - j) - 1;
    // helper variables based on current loop iteration
    const bool tile_first = (slice == 0);
    const bool tile_last = (slice == (ins_in.bits_l + ins_in.bits_r - 2));
    const bool rhstile_first = tile_first && (m == 0);
    const bool rhstile_last = tile_last && (m == ins_in.tiles_m-1);
    if(rhstile_first) {
      // when starting a new tile, wait for fetch stage to signal
      sync.isSendToken = 0;
      sync.chanID = 0;
      out.write(sync.asRaw());
      ap_wait();
    }
    if(tile_first) {
      // when starting a new tile, wait for fetch stage to signal
      sync.isSendToken = 0;
      sync.chanID = 0;
      out.write(sync.asRaw());
      ap_wait();
      // starting a new result tile:
      // acquire a result buffer
      sync.isRunCfg = 0;
      sync.isSendToken = 0;
      sync.chanID = 1;
      out.write(sync.asRaw());
      ap_wait();
    }
    // whether the current bit position is negative for
    // the input matrices
    const bool lbit_last = (l == ins_in.bits_l-1);
    const bool rbit_last = (r == ins_in.bits_r-1);
    const bool neg_l = lbit_last && ins_in.signed_l;
    const bool neg_r = rbit_last && ins_in.signed_r;
    bool negate = neg_l ^ neg_r;
    // TODO consider removing mults here, use mod counters
    const uint16_t offset_l = ins_in.tiles_k * l;
    const uint16_t offset_r = ins_in.tiles_k * r;
    exec.lhsOffset = (ins_in.base_l + lmem_region_offset + offset_l) << ETF_S;
    // note: no buffer regions for RHS tiles
    exec.rhsOffset = (ins_in.base_r + rmem_region_offset + offset_r) << ETF_S;
    exec.numTiles = ins_in.tiles_k;
    exec.shiftAmount = (j == slice - z2 ? 1 : 0);
    exec.negate = negate ? 1 : 0;
    // clear accumulator on first iteration of this result tile
    exec.clear_before_first_accumulation = tile_first ? 1 : 0;

    // write result on first iteration of this result tile
    exec.writeEn = tile_last ? 1 : 0;
    exec.writeAddr = ins_in.base_res + offset_res;
    out.write(exec.asRaw());
    ap_wait();
    if(tile_last) {
      // finished computing result tile
      // release the result buffer
      sync.isSendToken = 1;
      sync.chanID = 1;
      out.write(sync.asRaw());
      ap_wait();
      // iteration tracking logic: result buffer offset
      offset_res++;
      if(offset_res == 2) {
        offset_res = 0;
      }
      // when finishing with rhs tile, signal fetch stage to release buffer
      // release the input buffers
      sync.isSendToken = 1;
      sync.chanID = 0;
      out.write(sync.asRaw());
      ap_wait();
      // use the next rmem region for following fetch
      lmem_region++;
      lmem_region_offset += lmem_region_size;
      if(lmem_region == lmem_num_regions) {
        lmem_region = 0;
        lmem_region_offset = 0;
      }
    }
    if(rhstile_last) {
      // release buffer to fetch stage
      sync.isSendToken = 1;
      sync.chanID = 0;
      out.write(sync.asRaw());
      ap_wait();
      // use the next rmem region for following execution
      rmem_region++;
      rmem_region_offset += rmem_region_size;
      if(rmem_region == rmem_num_regions) {
        rmem_region = 0;
        rmem_region_offset = 0;
      }
    }

    // iteration tracking logic: nested loops over tiles and bits
    j--;
    if(j < z1) {
      slice++;
      z1 = slice < ins_in.bits_r ? 0 : slice - ins_in.bits_r + 1;
      z2 = slice < ins_in.bits_l ? 0 : slice - ins_in.bits_l + 1;
      j = slice - z2;
      if(slice == ins_in.bits_l + ins_in.bits_r - 1) {
        slice = 0;
        z1 = slice < ins_in.bits_r ? 0 : slice - ins_in.bits_r + 1;
        z2 = slice < ins_in.bits_l ? 0 : slice - ins_in.bits_l + 1;
        j = slice - z2;
        m++;
        if(m == ins_in.tiles_m) {
          m = 0;
          n++;
          if(n == ins_in.tiles_n) {
            n = 0;
          }
        }
      }
    }
  }
}
}

#include "ExecInstrGen_TemplateDefs.hpp"
void ExecInstrGen(
  hls::stream<ap_uint<BISMO_MMDESCR_BITS>> & in,
  hls::stream<ap_uint<BISMO_INSTR_BITS>> & out
) {
  #pragma HLS INTERFACE ap_ctrl_none port=return
  #pragma HLS INTERFACE axis port=out
  #pragma HLS INTERFACE axis port=in

  ExecInstrGen_RHSLHSTiling<
    TEMPLATE_PARAM_LMEM, TEMPLATE_PARAM_RMEM, TEMPLATE_PARAM_ETF_S
  >(in, out);
}
