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

void ResultInstrGen(
  hls::stream<ap_uint<BISMO_MMDESCR_BITS>> & in,
  hls::stream<ap_uint<BISMO_INSTR_BITS>> & out
);

void make_golden(hls::stream<ap_uint<BISMO_INSTR_BITS>> & out) {
  out.write(ap_uint<BISMO_INSTR_BITS>("02", 16));
  out.write(ap_uint<BISMO_INSTR_BITS>("050000007D00000000000000006", 16));
  out.write(ap_uint<BISMO_INSTR_BITS>("0A", 16));
  out.write(ap_uint<BISMO_INSTR_BITS>("02", 16));
  out.write(ap_uint<BISMO_INSTR_BITS>("050000007D88000000000000006", 16));
  out.write(ap_uint<BISMO_INSTR_BITS>("0A", 16));
  out.write(ap_uint<BISMO_INSTR_BITS>("02", 16));
  out.write(ap_uint<BISMO_INSTR_BITS>("050000007E00000000000000006", 16));
  out.write(ap_uint<BISMO_INSTR_BITS>("0A", 16));
  out.write(ap_uint<BISMO_INSTR_BITS>("02", 16));
  out.write(ap_uint<BISMO_INSTR_BITS>("050000007E88000000000000006", 16));
  out.write(ap_uint<BISMO_INSTR_BITS>("0A", 16));
  out.write(ap_uint<BISMO_INSTR_BITS>("02", 16));
  out.write(ap_uint<BISMO_INSTR_BITS>("050000007F00000000000000006", 16));
  out.write(ap_uint<BISMO_INSTR_BITS>("0A", 16));
  out.write(ap_uint<BISMO_INSTR_BITS>("02", 16));
  out.write(ap_uint<BISMO_INSTR_BITS>("050000007F88000000000000006", 16));
  out.write(ap_uint<BISMO_INSTR_BITS>("0A", 16));
  out.write(ap_uint<BISMO_INSTR_BITS>("02", 16));
  out.write(ap_uint<BISMO_INSTR_BITS>("050000008000000000000000006", 16));
  out.write(ap_uint<BISMO_INSTR_BITS>("0A", 16));
  out.write(ap_uint<BISMO_INSTR_BITS>("02", 16));
  out.write(ap_uint<BISMO_INSTR_BITS>("050000008088000000000000006", 16));
  out.write(ap_uint<BISMO_INSTR_BITS>("0A", 16));
  out.write(ap_uint<BISMO_INSTR_BITS>("02", 16));
  out.write(ap_uint<BISMO_INSTR_BITS>("050000008100000000000000006", 16));
  out.write(ap_uint<BISMO_INSTR_BITS>("0A", 16));
  out.write(ap_uint<BISMO_INSTR_BITS>("02", 16));
  out.write(ap_uint<BISMO_INSTR_BITS>("050000008188000000000000006", 16));
  out.write(ap_uint<BISMO_INSTR_BITS>("0A", 16));
  out.write(ap_uint<BISMO_INSTR_BITS>("02", 16));
  out.write(ap_uint<BISMO_INSTR_BITS>("050000008700000000000000006", 16));
  out.write(ap_uint<BISMO_INSTR_BITS>("0A", 16));
  out.write(ap_uint<BISMO_INSTR_BITS>("02", 16));
  out.write(ap_uint<BISMO_INSTR_BITS>("050000008788000000000000006", 16));
  out.write(ap_uint<BISMO_INSTR_BITS>("0A", 16));
  out.write(ap_uint<BISMO_INSTR_BITS>("02", 16));
  out.write(ap_uint<BISMO_INSTR_BITS>("050000008800000000000000006", 16));
  out.write(ap_uint<BISMO_INSTR_BITS>("0A", 16));
  out.write(ap_uint<BISMO_INSTR_BITS>("02", 16));
  out.write(ap_uint<BISMO_INSTR_BITS>("050000008888000000000000006", 16));
  out.write(ap_uint<BISMO_INSTR_BITS>("0A", 16));
  out.write(ap_uint<BISMO_INSTR_BITS>("02", 16));
  out.write(ap_uint<BISMO_INSTR_BITS>("050000008900000000000000006", 16));
  out.write(ap_uint<BISMO_INSTR_BITS>("0A", 16));
  out.write(ap_uint<BISMO_INSTR_BITS>("02", 16));
  out.write(ap_uint<BISMO_INSTR_BITS>("050000008988000000000000006", 16));
  out.write(ap_uint<BISMO_INSTR_BITS>("0A", 16));
  out.write(ap_uint<BISMO_INSTR_BITS>("02", 16));
  out.write(ap_uint<BISMO_INSTR_BITS>("050000008A00000000000000006", 16));
  out.write(ap_uint<BISMO_INSTR_BITS>("0A", 16));
  out.write(ap_uint<BISMO_INSTR_BITS>("02", 16));
  out.write(ap_uint<BISMO_INSTR_BITS>("050000008A88000000000000006", 16));
  out.write(ap_uint<BISMO_INSTR_BITS>("0A", 16));
  out.write(ap_uint<BISMO_INSTR_BITS>("02", 16));
  out.write(ap_uint<BISMO_INSTR_BITS>("050000008B00000000000000006", 16));
  out.write(ap_uint<BISMO_INSTR_BITS>("0A", 16));
  out.write(ap_uint<BISMO_INSTR_BITS>("02", 16));
  out.write(ap_uint<BISMO_INSTR_BITS>("050000008B88000000000000006", 16));
  out.write(ap_uint<BISMO_INSTR_BITS>("0A", 16));
  out.write(ap_uint<BISMO_INSTR_BITS>("02", 16));
  out.write(ap_uint<BISMO_INSTR_BITS>("050000009100000000000000006", 16));
  out.write(ap_uint<BISMO_INSTR_BITS>("0A", 16));
  out.write(ap_uint<BISMO_INSTR_BITS>("02", 16));
  out.write(ap_uint<BISMO_INSTR_BITS>("050000009188000000000000006", 16));
  out.write(ap_uint<BISMO_INSTR_BITS>("0A", 16));
  out.write(ap_uint<BISMO_INSTR_BITS>("02", 16));
  out.write(ap_uint<BISMO_INSTR_BITS>("050000009200000000000000006", 16));
  out.write(ap_uint<BISMO_INSTR_BITS>("0A", 16));
  out.write(ap_uint<BISMO_INSTR_BITS>("02", 16));
  out.write(ap_uint<BISMO_INSTR_BITS>("050000009288000000000000006", 16));
  out.write(ap_uint<BISMO_INSTR_BITS>("0A", 16));
  out.write(ap_uint<BISMO_INSTR_BITS>("02", 16));
  out.write(ap_uint<BISMO_INSTR_BITS>("050000009300000000000000006", 16));
  out.write(ap_uint<BISMO_INSTR_BITS>("0A", 16));
  out.write(ap_uint<BISMO_INSTR_BITS>("02", 16));
  out.write(ap_uint<BISMO_INSTR_BITS>("050000009388000000000000006", 16));
  out.write(ap_uint<BISMO_INSTR_BITS>("0A", 16));
  out.write(ap_uint<BISMO_INSTR_BITS>("02", 16));
  out.write(ap_uint<BISMO_INSTR_BITS>("050000009400000000000000006", 16));
  out.write(ap_uint<BISMO_INSTR_BITS>("0A", 16));
  out.write(ap_uint<BISMO_INSTR_BITS>("02", 16));
  out.write(ap_uint<BISMO_INSTR_BITS>("050000009488000000000000006", 16));
  out.write(ap_uint<BISMO_INSTR_BITS>("0A", 16));
  out.write(ap_uint<BISMO_INSTR_BITS>("02", 16));
  out.write(ap_uint<BISMO_INSTR_BITS>("050000009500000000000000006", 16));
  out.write(ap_uint<BISMO_INSTR_BITS>("0A", 16));
  out.write(ap_uint<BISMO_INSTR_BITS>("02", 16));
  out.write(ap_uint<BISMO_INSTR_BITS>("050000009588000000000000006", 16));
  out.write(ap_uint<BISMO_INSTR_BITS>("0A", 16));
  out.write(ap_uint<BISMO_INSTR_BITS>("02", 16));
  out.write(ap_uint<BISMO_INSTR_BITS>("050000009B00000000000000006", 16));
  out.write(ap_uint<BISMO_INSTR_BITS>("0A", 16));
  out.write(ap_uint<BISMO_INSTR_BITS>("02", 16));
  out.write(ap_uint<BISMO_INSTR_BITS>("050000009B88000000000000006", 16));
  out.write(ap_uint<BISMO_INSTR_BITS>("0A", 16));
  out.write(ap_uint<BISMO_INSTR_BITS>("02", 16));
  out.write(ap_uint<BISMO_INSTR_BITS>("050000009C00000000000000006", 16));
  out.write(ap_uint<BISMO_INSTR_BITS>("0A", 16));
  out.write(ap_uint<BISMO_INSTR_BITS>("02", 16));
  out.write(ap_uint<BISMO_INSTR_BITS>("050000009C88000000000000006", 16));
  out.write(ap_uint<BISMO_INSTR_BITS>("0A", 16));
  out.write(ap_uint<BISMO_INSTR_BITS>("02", 16));
  out.write(ap_uint<BISMO_INSTR_BITS>("050000009D00000000000000006", 16));
  out.write(ap_uint<BISMO_INSTR_BITS>("0A", 16));
  out.write(ap_uint<BISMO_INSTR_BITS>("02", 16));
  out.write(ap_uint<BISMO_INSTR_BITS>("050000009D88000000000000006", 16));
  out.write(ap_uint<BISMO_INSTR_BITS>("0A", 16));
  out.write(ap_uint<BISMO_INSTR_BITS>("02", 16));
  out.write(ap_uint<BISMO_INSTR_BITS>("050000009E00000000000000006", 16));
  out.write(ap_uint<BISMO_INSTR_BITS>("0A", 16));
  out.write(ap_uint<BISMO_INSTR_BITS>("02", 16));
  out.write(ap_uint<BISMO_INSTR_BITS>("050000009E88000000000000006", 16));
  out.write(ap_uint<BISMO_INSTR_BITS>("0A", 16));
  out.write(ap_uint<BISMO_INSTR_BITS>("02", 16));
  out.write(ap_uint<BISMO_INSTR_BITS>("050000009F00000000000000006", 16));
  out.write(ap_uint<BISMO_INSTR_BITS>("0A", 16));
  out.write(ap_uint<BISMO_INSTR_BITS>("02", 16));
  out.write(ap_uint<BISMO_INSTR_BITS>("050000009F88000000000000006", 16));
  out.write(ap_uint<BISMO_INSTR_BITS>("0A", 16));
  out.write(ap_uint<BISMO_INSTR_BITS>("02", 16));
  out.write(ap_uint<BISMO_INSTR_BITS>("05000000A500000000000000006", 16));
  out.write(ap_uint<BISMO_INSTR_BITS>("0A", 16));
  out.write(ap_uint<BISMO_INSTR_BITS>("02", 16));
  out.write(ap_uint<BISMO_INSTR_BITS>("05000000A588000000000000006", 16));
  out.write(ap_uint<BISMO_INSTR_BITS>("0A", 16));
  out.write(ap_uint<BISMO_INSTR_BITS>("02", 16));
  out.write(ap_uint<BISMO_INSTR_BITS>("05000000A600000000000000006", 16));
  out.write(ap_uint<BISMO_INSTR_BITS>("0A", 16));
  out.write(ap_uint<BISMO_INSTR_BITS>("02", 16));
  out.write(ap_uint<BISMO_INSTR_BITS>("05000000A688000000000000006", 16));
  out.write(ap_uint<BISMO_INSTR_BITS>("0A", 16));
  out.write(ap_uint<BISMO_INSTR_BITS>("02", 16));
  out.write(ap_uint<BISMO_INSTR_BITS>("05000000A700000000000000006", 16));
  out.write(ap_uint<BISMO_INSTR_BITS>("0A", 16));
  out.write(ap_uint<BISMO_INSTR_BITS>("02", 16));
  out.write(ap_uint<BISMO_INSTR_BITS>("05000000A788000000000000006", 16));
  out.write(ap_uint<BISMO_INSTR_BITS>("0A", 16));
  out.write(ap_uint<BISMO_INSTR_BITS>("02", 16));
  out.write(ap_uint<BISMO_INSTR_BITS>("05000000A800000000000000006", 16));
  out.write(ap_uint<BISMO_INSTR_BITS>("0A", 16));
  out.write(ap_uint<BISMO_INSTR_BITS>("02", 16));
  out.write(ap_uint<BISMO_INSTR_BITS>("05000000A888000000000000006", 16));
  out.write(ap_uint<BISMO_INSTR_BITS>("0A", 16));
  out.write(ap_uint<BISMO_INSTR_BITS>("02", 16));
  out.write(ap_uint<BISMO_INSTR_BITS>("05000000A900000000000000006", 16));
  out.write(ap_uint<BISMO_INSTR_BITS>("0A", 16));
  out.write(ap_uint<BISMO_INSTR_BITS>("02", 16));
  out.write(ap_uint<BISMO_INSTR_BITS>("05000000A988000000000000006", 16));
  out.write(ap_uint<BISMO_INSTR_BITS>("0A", 16));
  out.write(ap_uint<BISMO_INSTR_BITS>("02", 16));
  out.write(ap_uint<BISMO_INSTR_BITS>("05000000AF00000000000000006", 16));
  out.write(ap_uint<BISMO_INSTR_BITS>("0A", 16));
  out.write(ap_uint<BISMO_INSTR_BITS>("02", 16));
  out.write(ap_uint<BISMO_INSTR_BITS>("05000000AF88000000000000006", 16));
  out.write(ap_uint<BISMO_INSTR_BITS>("0A", 16));
  out.write(ap_uint<BISMO_INSTR_BITS>("02", 16));
  out.write(ap_uint<BISMO_INSTR_BITS>("05000000B000000000000000006", 16));
  out.write(ap_uint<BISMO_INSTR_BITS>("0A", 16));
  out.write(ap_uint<BISMO_INSTR_BITS>("02", 16));
  out.write(ap_uint<BISMO_INSTR_BITS>("05000000B088000000000000006", 16));
  out.write(ap_uint<BISMO_INSTR_BITS>("0A", 16));
  out.write(ap_uint<BISMO_INSTR_BITS>("02", 16));
  out.write(ap_uint<BISMO_INSTR_BITS>("05000000B100000000000000006", 16));
  out.write(ap_uint<BISMO_INSTR_BITS>("0A", 16));
  out.write(ap_uint<BISMO_INSTR_BITS>("02", 16));
  out.write(ap_uint<BISMO_INSTR_BITS>("05000000B188000000000000006", 16));
  out.write(ap_uint<BISMO_INSTR_BITS>("0A", 16));
  out.write(ap_uint<BISMO_INSTR_BITS>("02", 16));
  out.write(ap_uint<BISMO_INSTR_BITS>("05000000B200000000000000006", 16));
  out.write(ap_uint<BISMO_INSTR_BITS>("0A", 16));
  out.write(ap_uint<BISMO_INSTR_BITS>("02", 16));
  out.write(ap_uint<BISMO_INSTR_BITS>("05000000B288000000000000006", 16));
  out.write(ap_uint<BISMO_INSTR_BITS>("0A", 16));
  out.write(ap_uint<BISMO_INSTR_BITS>("02", 16));
  out.write(ap_uint<BISMO_INSTR_BITS>("05000000B300000000000000006", 16));
  out.write(ap_uint<BISMO_INSTR_BITS>("0A", 16));
  out.write(ap_uint<BISMO_INSTR_BITS>("02", 16));
  out.write(ap_uint<BISMO_INSTR_BITS>("05000000B388000000000000006", 16));
  out.write(ap_uint<BISMO_INSTR_BITS>("0A", 16));
  out.write(ap_uint<BISMO_INSTR_BITS>("02", 16));
  out.write(ap_uint<BISMO_INSTR_BITS>("05000000B900000000000000006", 16));
  out.write(ap_uint<BISMO_INSTR_BITS>("0A", 16));
  out.write(ap_uint<BISMO_INSTR_BITS>("02", 16));
  out.write(ap_uint<BISMO_INSTR_BITS>("05000000B988000000000000006", 16));
  out.write(ap_uint<BISMO_INSTR_BITS>("0A", 16));
  out.write(ap_uint<BISMO_INSTR_BITS>("02", 16));
  out.write(ap_uint<BISMO_INSTR_BITS>("05000000BA00000000000000006", 16));
  out.write(ap_uint<BISMO_INSTR_BITS>("0A", 16));
  out.write(ap_uint<BISMO_INSTR_BITS>("02", 16));
  out.write(ap_uint<BISMO_INSTR_BITS>("05000000BA88000000000000006", 16));
  out.write(ap_uint<BISMO_INSTR_BITS>("0A", 16));
  out.write(ap_uint<BISMO_INSTR_BITS>("02", 16));
  out.write(ap_uint<BISMO_INSTR_BITS>("05000000BB00000000000000006", 16));
  out.write(ap_uint<BISMO_INSTR_BITS>("0A", 16));
  out.write(ap_uint<BISMO_INSTR_BITS>("02", 16));
  out.write(ap_uint<BISMO_INSTR_BITS>("05000000BB88000000000000006", 16));
  out.write(ap_uint<BISMO_INSTR_BITS>("0A", 16));
  out.write(ap_uint<BISMO_INSTR_BITS>("02", 16));
  out.write(ap_uint<BISMO_INSTR_BITS>("05000000BC00000000000000006", 16));
  out.write(ap_uint<BISMO_INSTR_BITS>("0A", 16));
  out.write(ap_uint<BISMO_INSTR_BITS>("02", 16));
  out.write(ap_uint<BISMO_INSTR_BITS>("05000000BC88000000000000006", 16));
  out.write(ap_uint<BISMO_INSTR_BITS>("0A", 16));
  out.write(ap_uint<BISMO_INSTR_BITS>("02", 16));
  out.write(ap_uint<BISMO_INSTR_BITS>("05000000BD00000000000000006", 16));
  out.write(ap_uint<BISMO_INSTR_BITS>("0A", 16));
  out.write(ap_uint<BISMO_INSTR_BITS>("02", 16));
  out.write(ap_uint<BISMO_INSTR_BITS>("05000000BD88000000000000006", 16));
  out.write(ap_uint<BISMO_INSTR_BITS>("0A", 16));
  out.write(ap_uint<BISMO_INSTR_BITS>("02", 16));
  out.write(ap_uint<BISMO_INSTR_BITS>("05000000C300000000000000006", 16));
  out.write(ap_uint<BISMO_INSTR_BITS>("0A", 16));
  out.write(ap_uint<BISMO_INSTR_BITS>("02", 16));
  out.write(ap_uint<BISMO_INSTR_BITS>("05000000C388000000000000006", 16));
  out.write(ap_uint<BISMO_INSTR_BITS>("0A", 16));
  out.write(ap_uint<BISMO_INSTR_BITS>("02", 16));
  out.write(ap_uint<BISMO_INSTR_BITS>("05000000C400000000000000006", 16));
  out.write(ap_uint<BISMO_INSTR_BITS>("0A", 16));
  out.write(ap_uint<BISMO_INSTR_BITS>("02", 16));
  out.write(ap_uint<BISMO_INSTR_BITS>("05000000C488000000000000006", 16));
  out.write(ap_uint<BISMO_INSTR_BITS>("0A", 16));
  out.write(ap_uint<BISMO_INSTR_BITS>("02", 16));
  out.write(ap_uint<BISMO_INSTR_BITS>("05000000C500000000000000006", 16));
  out.write(ap_uint<BISMO_INSTR_BITS>("0A", 16));
  out.write(ap_uint<BISMO_INSTR_BITS>("02", 16));
  out.write(ap_uint<BISMO_INSTR_BITS>("05000000C588000000000000006", 16));
  out.write(ap_uint<BISMO_INSTR_BITS>("0A", 16));
  out.write(ap_uint<BISMO_INSTR_BITS>("02", 16));
  out.write(ap_uint<BISMO_INSTR_BITS>("05000000C600000000000000006", 16));
  out.write(ap_uint<BISMO_INSTR_BITS>("0A", 16));
  out.write(ap_uint<BISMO_INSTR_BITS>("02", 16));
  out.write(ap_uint<BISMO_INSTR_BITS>("05000000C688000000000000006", 16));
  out.write(ap_uint<BISMO_INSTR_BITS>("0A", 16));
  out.write(ap_uint<BISMO_INSTR_BITS>("02", 16));
  out.write(ap_uint<BISMO_INSTR_BITS>("05000000C700000000000000006", 16));
  out.write(ap_uint<BISMO_INSTR_BITS>("0A", 16));
  out.write(ap_uint<BISMO_INSTR_BITS>("02", 16));
  out.write(ap_uint<BISMO_INSTR_BITS>("05000000C788000000000000006", 16));
  out.write(ap_uint<BISMO_INSTR_BITS>("0A", 16));
  out.write(ap_uint<BISMO_INSTR_BITS>("010000000000004000000000000006", 16));
}

bool TestResultInstrGen() {
  cout << "Now running HLS Test for ResultInstrGen" << endl;
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
  desc.dram_res = 2000;
  in.write(desc.asRaw());
  make_golden(golden);
  ResultInstrGen(in, out);
  bool all_OK = true;
  if(out.size() != golden.size()) {
    cout << "ERROR: Incorrect number of Result instructions produced!" << endl;
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
  if(TestResultInstrGen()) {
    cout << "Test passed: ResultInstrGen" << endl;
    return 0;
  } else {
    cout << "Test failed: ResultInstrGen" << endl;
    return -1;
  }
}
