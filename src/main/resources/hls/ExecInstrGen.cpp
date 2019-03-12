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

void ExecInstrGen(
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

  SingleMMDescriptor ins_in;
  ins_in.fromRaw(in.read());
  ap_wait();
  sync.targetStage = stgExec;
  exec.targetStage = stgExec;
  exec.isRunCfg = 1;
  sync.isRunCfg = 0;
  // start by acquiring input buffers
  sync.isSendToken = 0;
  sync.chanID = 0;
  out.write(sync.asRaw());
  ap_wait();
  // compute the size of the iteration space
  const size_t total_iters = ins_in.tiles_m * ins_in.tiles_n * ins_in.bits_l * ins_in.bits_r;
  /// iteration variables
  uint16_t m = 0, n = 0;
  uint8_t l = 0, r = 0;
  uint8_t offset_res = 0;
  // single iteration space for the entire instrgen
  for(size_t i = 0; i < total_iters; i++) {
    #pragma HLS PIPELINE II=1
    // helper variables based on current loop iteration
    const bool tile_first = (l == 0) && (r == 0);
    const bool tile_last = (l == ins_in.bits_l-1) && (r == ins_in.bits_r-1);
    if(tile_first) {
      // starting a new result tile:
      // acquire a result buffer
      sync.isRunCfg = 0;
      sync.isSendToken = 0;
      sync.chanID = 1;
      out.write(sync.asRaw());
      ap_wait();
    }
    const uint8_t weight = l + r;
    // whether the current bit position is negative for
    // the input matrices
    const bool lbit_last = (l == ins_in.bits_l-1);
    const bool rbit_last = (r == ins_in.bits_r-1);
    const bool neg_l = lbit_last && ins_in.signed_l;
    const bool neg_r = rbit_last && ins_in.signed_r;
    bool negate = neg_l ^ neg_r;
    // TODO consider removing mults here, use mod counters
    // offset_l = ins_in.tiles_k * m + ins_in.tiles_k * ins_in.tiles_m * l;
    // offset_r = ins_in.tiles_k * n + ins_in.tiles_k * ins_in.tiles_n * r;
    const uint16_t offset_l = ins_in.tiles_k * (m + l * ins_in.tiles_m);
    const uint16_t offset_r = ins_in.tiles_k * (n + r * ins_in.tiles_n);
    exec.lhsOffset = ins_in.base_l + offset_l;
    exec.rhsOffset = ins_in.base_r + offset_r;
    exec.numTiles = ins_in.tiles_k;
    exec.shiftAmount = weight;
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
      if(offset_res == ins_in.nbufs_res) {
        offset_res = 0;
      }
    }
    // iteration tracking logic: nested loops over tiles and bits
    r++;
    if(r == ins_in.bits_r) {
      r = 0;
      l++;
      if(l == ins_in.bits_l) {
        l = 0;
        n++;
        if(n == ins_in.tiles_n) {
          n = 0;
          m++;
          if(m == ins_in.tiles_m) {
            m = 0;
          }
        }
      }
    }
  }
  // release the input buffers
  sync.isSendToken = 1;
  sync.chanID = 0;
  out.write(sync.asRaw());
}
}

void ExecInstrGen_RHSTiling(
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
  uint8_t l = 0, r = 0;
  uint8_t offset_res = 0;
  // single iteration space for the entire instrgen
  for(size_t i = 0; i < total_iters; i++) {
    #pragma HLS PIPELINE II=1
    // helper variables based on current loop iteration
    const bool tile_first = (l == 0) && (r == 0);
    const bool tile_last = (l == ins_in.bits_l-1) && (r == ins_in.bits_r-1);
    const bool rhstile_first = tile_first && (m == 0);
    const bool rhstile_last = tile_last && (m == ins_in.tiles_m-1);
    // when starting a new rhs tile, wait for fetch stage to signal
    if(rhstile_first) {
      sync.isSendToken = 0;
      sync.chanID = 0;
      out.write(sync.asRaw());
      ap_wait();
    }
    if(tile_first) {
      // starting a new result tile:
      // acquire a result buffer
      sync.isRunCfg = 0;
      sync.isSendToken = 0;
      sync.chanID = 1;
      out.write(sync.asRaw());
      ap_wait();
    }
    const uint8_t weight = l + r;
    // whether the current bit position is negative for
    // the input matrices
    const bool lbit_last = (l == ins_in.bits_l-1);
    const bool rbit_last = (r == ins_in.bits_r-1);
    const bool neg_l = lbit_last && ins_in.signed_l;
    const bool neg_r = rbit_last && ins_in.signed_r;
    bool negate = neg_l ^ neg_r;
    // TODO consider removing mults here, use mod counters
    // TODO tweak base addresses for F-E concurrency
    const uint16_t offset_l = ins_in.tiles_k * (m + l * ins_in.tiles_m);
    // note that we don't use tiles_n and n here -- always operate on a single
    // bit stripe of rhs while on chip
    const uint16_t offset_r = ins_in.tiles_k * r;
    exec.lhsOffset = ins_in.base_l + offset_l;
    exec.rhsOffset = ins_in.base_r + offset_r;
    exec.numTiles = ins_in.tiles_k;
    exec.shiftAmount = weight;
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
      if(offset_res == ins_in.nbufs_res) {
        offset_res = 0;
      }
    }

    // when finishing with rhs tile, signal fetch stage to release buffer
    if(rhstile_last) {
      // release the input buffers
      sync.isSendToken = 1;
      sync.chanID = 0;
      out.write(sync.asRaw());
      ap_wait();
    }
    // iteration tracking logic: nested loops over tiles and bits
    r++;
    if(r == ins_in.bits_r) {
      r = 0;
      l++;
      if(l == ins_in.bits_l) {
        l = 0;
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
