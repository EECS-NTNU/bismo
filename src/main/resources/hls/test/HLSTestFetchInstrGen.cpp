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
#include <hls_stream.h>
#include "BISMOInstruction.hpp"
#include <iostream>

using namespace std;

void FetchInstrGen(
  hls::stream<ap_uint<BISMO_MMDESCR_BITS>> & in,
  hls::stream<ap_uint<BISMO_INSTR_BITS>> & out
);

void make_golden(hls::stream<ap_uint<BISMO_INSTR_BITS>> & out) {
  out.write(ap_uint<BISMO_INSTR_BITS>("0", 16));
  out.write(ap_uint<BISMO_INSTR_BITS>("08030002000040000003E800008084", 16));
  out.write(ap_uint<BISMO_INSTR_BITS>("08", 16));
  out.write(ap_uint<BISMO_INSTR_BITS>("0", 16));
  out.write(ap_uint<BISMO_INSTR_BITS>("080300020000400000042802008084", 16));
  out.write(ap_uint<BISMO_INSTR_BITS>("08", 16));
  out.write(ap_uint<BISMO_INSTR_BITS>("0", 16));
  out.write(ap_uint<BISMO_INSTR_BITS>("080300020000400000046804008084", 16));
  out.write(ap_uint<BISMO_INSTR_BITS>("08", 16));
  out.write(ap_uint<BISMO_INSTR_BITS>("0", 16));
  out.write(ap_uint<BISMO_INSTR_BITS>("08030002000040000004A806008084", 16));
  out.write(ap_uint<BISMO_INSTR_BITS>("08", 16));
  out.write(ap_uint<BISMO_INSTR_BITS>("0", 16));
  out.write(ap_uint<BISMO_INSTR_BITS>("08030002000040000004E800008084", 16));
  out.write(ap_uint<BISMO_INSTR_BITS>("08", 16));
  out.write(ap_uint<BISMO_INSTR_BITS>("0", 16));
  out.write(ap_uint<BISMO_INSTR_BITS>("080300020000400000052802008084", 16));
  out.write(ap_uint<BISMO_INSTR_BITS>("08", 16));
  out.write(ap_uint<BISMO_INSTR_BITS>("0", 16));
  out.write(ap_uint<BISMO_INSTR_BITS>("080300020000400000056804008084", 16));
  out.write(ap_uint<BISMO_INSTR_BITS>("08", 16));
  out.write(ap_uint<BISMO_INSTR_BITS>("0", 16));
  out.write(ap_uint<BISMO_INSTR_BITS>("08030002000040000005A806008084", 16));
  out.write(ap_uint<BISMO_INSTR_BITS>("08", 16));
}

bool TestFetchInstrGen() {
  cout << "Now running HLS Test for FetchInstrGen" << endl;
  hls::stream<ap_uint<BISMO_MMDESCR_BITS>> in;
  hls::stream<ap_uint<BISMO_INSTR_BITS>> out;
  hls::stream<ap_uint<BISMO_INSTR_BITS>> golden;
  SingleMMDescriptor desc;
  BISMOInstruction ins, golden_ins;
  desc.tiles_m = 10;
  desc.tiles_k = 4;
  desc.tiles_n = 8;
  desc.bits_l = 2;
  desc.bits_r = 3;
  desc.base_l = 0;
  desc.base_r = 0;
  desc.nbufs_fetch_exec_log2 = 2;
  desc.dram_lhs = 0;
  desc.dram_rhs = 1000;
  in.write(desc.asRaw());
  make_golden(golden);
  FetchInstrGen(in, out);

  bool all_OK = true;
  if(out.size() != golden.size()) {
    cout << "ERROR: Incorrect number of fetch instructions produced!" << endl;
    all_OK = false;
  }

  while(!out.empty()) {
    ins = out.read();
    golden_ins = golden.read();
    all_OK &= (ins == golden_ins);
    if(ins != golden_ins) {
      cout << "ERROR: Mismatch found. Expected: " << golden_ins << endl;
      cout << "Found: " << ins << endl;
    }
  }
  return all_OK;
}

int main(int argc, char *argv[]) {
  if(TestFetchInstrGen()) {
    cout << "Test passed: FetchInstrGen" << endl;
    return 0;
  } else {
    cout << "Test failed: FetchInstrGen" << endl;
    return -1;
  }
}
