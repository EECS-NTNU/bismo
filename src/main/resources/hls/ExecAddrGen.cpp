/*******************************************************************************
# Copyright (c) 2019, Xilinx, Inc.
# All rights reserved.
# Author: Yaman Umuroglu, Davide Conficconi
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

#define BISMO_EXECADDRSTRUCT_BITS 43

struct ExecAddr {
  ap_uint<16> lhsAddr;
  ap_uint<16> rhsAddr;
  ap_uint<1> rhsIsPadding;
  ap_uint<1> last;
  ap_uint<1> clear;
  ap_uint<1> negate;
  ap_uint<5> shift;
  ap_uint<1> writeEn;
  ap_uint<1> writeAddr;

  ap_uint<BISMO_EXECADDRSTRUCT_BITS> asRaw() {
    ap_uint<BISMO_EXECADDRSTRUCT_BITS> ret = 0;
    ret(15, 0) = lhsAddr;
    ret(31, 16) = rhsAddr;
    ret(32, 32) = rhsIsPadding;
    ret(33, 33) = last;
    ret(34, 34) = clear;
    ret(35, 35) = negate;
    ret(40, 36) = shift;
    ret(41, 41) = writeEn;
    ret(42, 42) = writeAddr;
    return ret;
  }

  void fromRaw(ap_uint<BISMO_EXECADDRSTRUCT_BITS> raw) {
    lhsAddr = raw(15, 0);
    rhsAddr = raw(31, 16);
    rhsIsPadding = raw(32, 32);
    last = raw(33, 33);
    clear = raw(34, 34);
    negate = raw(35, 35);
    shift = raw(40, 36);
    writeEn = raw(41, 41);
    writeAddr = raw(42, 42);
  }

  ExecAddr() {
    lhsAddr = 0;
    rhsAddr = 0;
    rhsIsPadding = 0;
    last = 0;
    clear = 0;
    negate = 0;
    shift = 0;
    writeEn = 0;
    writeAddr = 0;
  }
};

template <
  size_t ADDR_UNIT,
  size_t IMG_SIZE_BITWIDTH,
  size_t KRNL_SIZE_BITWIDTH,
  size_t STRIDE_BITWIDTH,
  size_t PADDING_BITWIDTH,
  size_t OUT_ADDR_BITWIDTH,
  size_t CONSTANT_ADDRESS
>
void ExecAddrGen_Templated(
  hls::stream<ap_uint<BISMO_INSTR_BITS>> & in,  // exec stage instructions
  hls::stream<ap_uint<BISMO_EXECADDRSTRUCT_BITS>> & out // generated addresses for access
) {
  #pragma HLS INTERFACE ap_ctrl_none port=return
  #pragma HLS INTERFACE axis port=out
  #pragma HLS INTERFACE axis port=in

  ExecAddr addr;
  // read an execute instruction from the input stream
  BISMOExecRunInstruction ins;
  ins.fromRaw(in.read());
  addr.shift = ins.shiftAmount;
  addr.negate = ins.negate;
  addr.writeEn = ins.writeEn;
  addr.writeAddr = ins.writeAddr;

  if(ins.cnvAddrGenMode == 1) {
    // use convolution mode for rhs, sequential for lhs
    const ap_uint<IMG_SIZE_BITWIDTH + PADDING_BITWIDTH> extended_dim = ins.cnvImgSize + (ins.cnvPad << 1);
    int i = 0, j = 0, kr = 0, kc = 0 ;
    ap_uint<16> seq = 0;
    bool sliding_finished = false;
    for(int iter = 0; !sliding_finished; iter++) {
      #pragma HLS PIPELINE II=1
      bool kernel_inbound = (i-1 + ins.cnvKernelSize < extended_dim) && (j-1 + ins.cnvKernelSize < extended_dim);
  		bool isNotPadding = ((ins.cnvPad - 1) < (kr+i) && (kr+i) < (ins.cnvImgSize + ins.cnvPad)) && ((ins.cnvPad - 1) < (kc+j) && (kc+j) < (ins.cnvImgSize + ins.cnvPad));
  		if(kernel_inbound) {
        addr.rhsIsPadding = isNotPadding ? 0 : 1;
        if(isNotPadding) {
          addr.rhsAddr = ((kr+i - ins.cnvPad) * (ins.cnvImgSize) + (kc+j - ins.cnvPad));
        } else {
          addr.rhsAddr = CONSTANT_ADDRESS;
        }
        // check boundaries to keep track of loop variables
  			kc++;
  			if(kc == ins.cnvKernelSize) {
  				kc = 0;
  				kr++;
  				if(kr == ins.cnvKernelSize) {
  					kr = 0;
  					j = j + ins.cnvStride;
  					if(j == extended_dim) {
  						j = 0;
  						i = i + ins.cnvStride;
  						sliding_finished = i == extended_dim;
  					}
  				}
  			}
        // finalize the output address packet
        // sequential memory access pattern on lhs
        addr.lhsAddr = seq;
        // TODO need to cater for ADDR_UNIT for convolutions here
        // add specified offsets to addresses
        addr.rhsAddr += ins.rhsOffset;
        addr.lhsAddr += ins.lhsOffset;
        addr.last = sliding_finished ? 1 : 0;
        addr.clear = ins.clear_before_first_accumulation & (seq == 0);
        out.write(addr.asRaw());
        seq++;
      } else {
        kr = 0;
  			kc = 0;
  			j = j + ins.cnvStride;
  			if(j == extended_dim) {
  				j = 0;
  				i = i + ins.cnvStride;
  				sliding_finished = i == extended_dim;
  			}
      }
    }
  } else {
    // use sequential access mode for both memories
    addr.lhsAddr = ins.lhsOffset;
    addr.rhsAddr = ins.rhsOffset;
    for(ap_uint<16> i = 0; i < ins.numTiles; i += 1) {
      // produce one address every cycle
      #pragma HLS PIPELINE II=1
      addr.rhsIsPadding = 0;
      addr.last = (ins.numTiles - i == 1);
      addr.clear = ins.clear_before_first_accumulation & (i == 0);
      addr.shift = ins.shiftAmount & (i == 0);
      out.write(addr.asRaw());
      addr.lhsAddr += ADDR_UNIT;
      addr.rhsAddr += ADDR_UNIT;
    }
  }
}

#include "ExecAddrGen_TemplateDefs.hpp"
void ExecAddrGen(
  hls::stream<ap_uint<BISMO_INSTR_BITS>> & in,  // exec stage instructions
  hls::stream<ap_uint<BISMO_EXECADDRSTRUCT_BITS>> & out // generated addresses for access
) {
  #pragma HLS INTERFACE ap_ctrl_none port=return
  #pragma HLS INTERFACE axis port=out
  #pragma HLS INTERFACE axis port=in

  ExecAddrGen_Templated<
  TEMPLATE_PARAM_ADDR_UNIT, TEMPLATE_PARAM_IMG_SIZE_BITWIDTH,
  TEMPLATE_PARAM_KRNL_SIZE_BITWIDTH, TEMPLATE_PARAM_STRIDE_BITWIDTH,
  TEMPLATE_PARAM_PADDING_BITWIDTH, TEMPLATE_PARAM_OUT_ADDR_BITWIDTH,
  TEMPLATE_PARAM_CONSTANT_ADDRESS
  >(
    in, out
  );
}
