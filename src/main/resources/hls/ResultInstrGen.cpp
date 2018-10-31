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
#include <hls_stream.h>
#include <stdint.h>
#include "BISMOInstruction.hpp"

template <
  // matmul array dimensions: rows, cols (note: no common)
  size_t M, size_t N,
  // bits per accumulator
  size_t A
>
void ResultInstrGen_Templated(
  hls::stream<ap_uint<BISMO_MMDESCR_BITS>> & in,
  hls::stream<ap_uint<BISMO_INSTR_BITS>> & out
) {
  #pragma HLS INTERFACE ap_ctrl_none port=return
  #pragma HLS INTERFACE axis port=out
  #pragma HLS INTERFACE axis port=in

  BISMOResultRunInstruction res;
  BISMOSyncInstruction sync;

  // set the invariants (values do not depend on loop iter)
  sync.targetStage = stgResult;
  res.targetStage = stgResult;
  res.isRunCfg = 1;
  sync.isRunCfg = 0;
  // read the descriptor
  SingleMMDescriptor ins_in;
  ins_in.fromRaw(in.read());

  // start by acquiring buffer from execute stage
  // receive token from execute stage
  sync.isSendToken = 0;
  sync.chanID = 0;
  out.write(sync.asRaw());

  const size_t bytes_per_acc = A / 8;
  const size_t bytes_per_res_tile = M * N * bytes_per_acc;

  // TODO expand to support multi-tile
  res.nop = 0;
  res.resmem_addr = 0;
  res.dram_base = ins_in.dram_res;
  res.dram_skip = bytes_per_acc * M; // TODO multitile will need matrix dim
  res.waitCompleteBytes = bytes_per_res_tile;
  // emit res instruction
  out.write(res.asRaw());

  // signal that res buffer is now free to be recycled
  // send token to execute stage
  sync.isSendToken = 1;
  sync.chanID = 0;
  out.write(sync.asRaw());
}

#include "ResultInstrGen_TemplateDefs.hpp"
void ResultInstrGen(
  hls::stream<ap_uint<BISMO_MMDESCR_BITS>> & in,
  hls::stream<ap_uint<BISMO_INSTR_BITS>> & out
) {
  #pragma HLS INTERFACE ap_ctrl_none port=return
  #pragma HLS INTERFACE axis port=out
  #pragma HLS INTERFACE axis port=in

  ResultInstrGen_Templated<
    TEMPLATE_PARAM_M, TEMPLATE_PARAM_N, TEMPLATE_PARAM_A
  >(
    in, out
  );
}
