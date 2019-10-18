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


bool TestFetchInstrGen() {
  cout << "Now running HLS Test for FetchInstrGen" << endl;
  hls::stream<ap_uint<BISMO_MMDESCR_BITS>> in;
  hls::stream<ap_uint<BISMO_INSTR_BITS>> out;
  SingleMMDescriptor desc;
  BISMOInstruction ins;
  desc.tiles_m = 6;
  desc.tiles_k = 4;
  desc.tiles_n = 12;
  desc.bits_l = 2;
  desc.bits_r = 3;
  desc.base_l = 0;
  desc.base_r = 0;
  desc.nbufs_fetch_exec_log2 = 2;
  desc.dram_lhs = 0;
  desc.dram_rhs = 1000;
  in.write(desc.asRaw());
  FetchInstrGen(in, out);

  bool all_OK = true;
  int correct_size = desc.tiles_m * 3 + (desc.tiles_n - 1) * 2 * desc.tiles_m + desc.tiles_n * 3 ;
  if(out.size() != correct_size) {
    cout << "ERROR: Incorrect number of fetch instructions produced!" << endl;
    cout << "Expected" << correct_size << endl;
    cout << "Found" << out.size() << endl;
    all_OK = false;
  }

  while(!out.empty()) {
    ins = out.read();
    cout << "Found: " << ins << endl;
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
