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
  // matmul array dimensions: rows, cols (note: no common)
  size_t M, size_t N,
  // bits per accumulator
  size_t A
>
void ResultInstrGen_RHSTiling_Templated(
  hls::stream<ap_uint<BISMO_MMDESCR_BITS>> & in,
  hls::stream<ap_uint<BISMO_INSTR_BITS>> & out
) {
  #pragma HLS INTERFACE ap_ctrl_none port=return
  #pragma HLS INTERFACE axis port=out
  #pragma HLS INTERFACE axis port=in
io_section:{
  #pragma HLS protocol fixed

  BISMOResultRunInstruction res;
  BISMOSyncInstruction sync;

  const size_t bytes_per_acc = A / 8;
  const size_t bytes_per_res_tile = M * N * bytes_per_acc;

  // set the invariants (values do not depend on loop iter)
  sync.targetStage = stgResult;
  res.targetStage = stgResult;
  res.isRunCfg = 1;
  sync.isRunCfg = 0;
  // read the descriptor
  SingleMMDescriptor ins_in;
  ins_in.fromRaw(in.read());
  ap_wait();

  // compute the size of the iteration space
  const size_t total_iters = ins_in.tiles_m * ins_in.tiles_n;
  /// iteration variables
  uint16_t m = 0, n = 0;
  uint8_t offset_res = 0;
  const size_t lhs_nrows_a = ins_in.tiles_m * M;
  const size_t dram_skip = bytes_per_acc * lhs_nrows_a;
  // single iteration space for the entire instrgen
  for(size_t i = 0; i < total_iters; i++) {
    // start by acquiring buffer from execute stage
    // receive token from execute stage
    sync.isSendToken = 0;
    sync.chanID = 0;
    out.write(sync.asRaw());
    ap_wait();
    // calculate the write offset
    // TODO optimize resource usage here by using adds inside iter. tracking
    uint32_t lhs_ind = M * m;
    uint32_t rhs_ind = N * n;
    size_t ind = rhs_ind * lhs_nrows_a + lhs_ind;
    res.dram_base = ins_in.dram_res + (ind * bytes_per_acc);
    res.resmem_addr = offset_res;
    res.dram_skip = dram_skip;
    res.nop = 0;
    res.waitCompleteBytes = 0;
    // emit res instruction
    out.write(res.asRaw());
    ap_wait();
    // update the result buffer offset
    offset_res++;
    // TODO: pass #exec-res buffers as template parameter as well
    if(offset_res == 2) {
      offset_res = 0;
    }
    // signal that res buffer is now free to be recycled
    // send token to execute stage
    sync.isSendToken = 1;
    sync.chanID = 0;
    out.write(sync.asRaw());
    ap_wait();
    // iteration tracking logic: nested loops over tiles
    m++;
    if(m == ins_in.tiles_m) {
      m = 0;
      n++;
      if(n == ins_in.tiles_n) {
        n = 0;
      }
    }
  }
  // generate a final instruction to ensure all writes completed
  res.nop = 1;
  res.waitCompleteBytes = 1;
  res.dram_base = 0;
  res.dram_skip = 0;
  res.resmem_addr = 0;
  out.write(res.asRaw());
}
}

#include "ResultInstrGen_TemplateDefs.hpp"
void ResultInstrGen(
  hls::stream<ap_uint<BISMO_MMDESCR_BITS>> & in,
  hls::stream<ap_uint<BISMO_INSTR_BITS>> & out
) {
  #pragma HLS INTERFACE ap_ctrl_none port=return
  #pragma HLS INTERFACE axis port=out
  #pragma HLS INTERFACE axis port=in

  ResultInstrGen_RHSTiling_Templated<
    TEMPLATE_PARAM_M, TEMPLATE_PARAM_N, TEMPLATE_PARAM_A
  >(
    in, out
  );
}
