# /*******************************************************************************
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

typedef struct {
  // number of tiles in a single binary matrix
  // expressed in terms of the instantiated DPA size
  uint16_t tiles_m;
  uint16_t tiles_k;
  uint16_t tiles_n;
  // number of bits in input matrices
  uint8_t bits_l;
  uint8_t bits_r;
  // signedness for the input matrices
  bool signed_l;
  bool signed_r;
  // base addresses for buffer accesses
  uint16_t base_l;
  uint16_t base_r;
  uint8_t base_res;
  // number of buffers for latency hiding
  uint8_t nbufs_res;
} SingleMMDescriptor;

void ExecInstrGen(
  hls::stream<SingleMMDescriptor> & desc_in,
  hls::stream<ap_uint<128>> & out
) {
  #pragma HLS INTERFACE ap_ctrl_none port=return
  #pragma HLS INTERFACE axis port=out
  #pragma HLS INTERFACE axis port=desc_in
  #pragma HLS DATA_PACK variable=desc_in

  BISMOExecRunInstruction exec;
  BISMOSyncInstruction sync;
  #pragma HLS DATA_PACK variable=exec
  #pragma HLS DATA_PACK variable=sync

  SingleMMDescriptor in = desc_in.read();
  // TODO: convert the software ExecInstrGenSingleMM to an HLS-friendly
  // format and call directly instead of copy-paste
  // all instructions are targeting the execute stage
  sync.targetStage = stgExec;
  // start by acquiring input buffers
  sync.isRunCfg = 0;
  sync.isSendToken = 0;
  sync.chanID = 0;
  out.write(sync.asRaw());
  // keep track of which result buffer we wrote to last
  size_t offset_res = 0;
  for(size_t m = 0; m < in.tiles_m; m++) {
    for(size_t n = 0; n < in.tiles_n; n++) {
      // starting a new result tile:
      // acquire a result buffer
      sync.isRunCfg = 0;
      sync.isSendToken = 0;
      sync.chanID = 1;
      out.write(sync.asRaw());
      for(size_t l = 0; l < in.bits_l; l++) {
        for(size_t r = 0; r < in.bits_r; r++) {
          // helper variables based on current loop iteration
          bool tile_first = (l == 0) && (r == 0);
          bool tile_last = (l == in.bits_l-1) && (r == in.bits_r-1);
          size_t weight = l + r;
          // whether the current bit position is negative for
          // the input matrices
          bool neg_l = (l == in.bits_l-1) && in.signed_l;
          bool neg_r = (r == in.bits_r-1) && in.signed_r;
          bool negate = neg_l ^ neg_r;
          size_t offset_l = in.tiles_k * (m + l * in.tiles_m);
          size_t offset_r = in.tiles_k * (n + r * in.tiles_n);
          // switch result buffers for latency hiding
          offset_res = (offset_res + 1) % in.nbufs_res;
          exec.isRunCfg = 1;
          exec.lhsOffset = in.base_l + offset_l;
          exec.rhsOffset = in.base_r + offset_r;
          exec.numTiles = in.tiles_k;
          exec.shiftAmount = weight;
          exec.negate = negate ? 1 : 0;
          // clear accumulator on first iteration of this result tile
          exec.clear_before_first_accumulation = tile_first ? 1 : 0;
          // write result on first iteration of this result tile
          exec.writeEn = tile_last ? 1 : 0;
          exec.writeAddr = in.base_res + offset_res;
          out.write(exec.asRaw());
        }
      }
      // finished computing result tile
      // release the result buffer
      sync.isRunCfg = 0;
      sync.isSendToken = 1;
      sync.chanID = 1;
      out.write(sync.asRaw());
    }
  }
  // release the input buffers
  sync.isRunCfg = 0;
  sync.isSendToken = 1;
  sync.chanID = 0;
  out.write(sync.asRaw());
}

void VerifyExecInstrEncoding(
  hls::stream<ap_uint<128>> & out
) {
  #pragma HLS INTERFACE ap_ctrl_none port=return
  #pragma HLS INTERFACE axis port=out

  BISMOExecRunInstruction exec;
  BISMOSyncInstruction sync;
  #pragma HLS DATA_PACK variable=exec
  #pragma HLS DATA_PACK variable=sync

  sync.isRunCfg = 0;
  sync.targetStage = 1;
  sync.isSendToken = 1;
  sync.chanID = 2;
  out.write(sync.asRaw());

  for(unsigned int i = 0; i < 10; i++) {
    #pragma HLS PIPELINE II=1
    exec.isRunCfg = 1;
    exec.targetStage = 1;
    exec.lhsOffset = i;
    exec.rhsOffset = 10 - i;
    exec.numTiles = 2 * i;
    exec.shiftAmount = i+1;
    out.write(exec.asRaw());
  }

  sync.isRunCfg = 0;
  sync.targetStage = 1;
  sync.isSendToken = 0;
  sync.chanID = 2;
  out.write(sync.asRaw());

}
