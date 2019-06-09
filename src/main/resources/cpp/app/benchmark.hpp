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

#include <iostream>
using namespace std;
#include "gemmbitserial/test/testhelpers.hpp"
#include "bismo_rt.hpp"

const char * delimiter = ", ";

void printInstrumentationHeaders(bismo_rt::InstrumentationData & data) {
  bismo_rt::InstrumentationData::iterator it;
  for(it = data.begin(); it != data.end(); it++) {
    cout << it->first << delimiter;
  }
  cout << endl;
}

void printInstrumentationData(bismo_rt::InstrumentationData & data) {
  bismo_rt::InstrumentationData::iterator it;
  for(it = data.begin(); it != data.end(); it++) {
    cout << it->second << delimiter;
  }
  cout << endl;
}

bismo_rt::InstrumentationData run_benchmark_matmul(
  size_t nrows_lhs, size_t nrows_rhs, size_t ncols, size_t nbits_lhs,
  size_t nbits_rhs
) {
  uint8_t * lhs = new uint8_t[nrows_lhs * ncols];
  uint8_t * rhs = new uint8_t[nrows_rhs * ncols];
  gemmbitserial::generateRandomVector(nbits_lhs, nrows_lhs*ncols, lhs);
  gemmbitserial::generateRandomVector(nbits_rhs, nrows_rhs*ncols, rhs);
  bismo_rt::MatMulDescriptor dscr;
  dscr.wbits = nbits_lhs;
  dscr.ibits = nbits_rhs;
  dscr.wsigned = false;
  dscr.isigned = false;
  dscr.M = nrows_lhs;
  dscr.K = ncols;
  dscr.N = nrows_rhs;
  bismo_rt::init();
  bismo_rt::InstrumentationData ret;
  try {
    bismo_rt::LayerHandle id = bismo_rt::initMatMul(dscr);
    uint8_t * accel_lhs = bismo_rt::getLayerLHSBuffer(id);
    uint8_t * accel_rhs = bismo_rt::getLayerRHSBuffer(id);
    int32_t * accel_res = bismo_rt::getLayerResBuffer(id);
    memcpy(accel_lhs, lhs, nrows_lhs * ncols);
    bismo_rt::syncLayerLHSBuffer(id);
    memcpy(accel_rhs, rhs, nrows_rhs * ncols);
    bismo_rt::syncLayerRHSBuffer(id);
    bismo_rt::execMatMul(id);
    bismo_rt::syncLayerResBuffer(id);
    ret = bismo_rt::getInstrumentationData(id);
  } catch(const char * e) {
    cout << "Exception: " << e << endl;
  }

  bismo_rt::deinit();

  delete [] lhs;
  delete [] rhs;
  return ret;
}

void benchmark_gemm_interactive() {
  while(1) {
    int rows, depth, cols, lhsbits, rhsbits;
    cout << "Enter rows depth cols, 0 to exit " << endl;
    cin >> rows;
    if(rows == 0) {
      return;
    }
    cin >> depth >> cols;
    cout << "Enter lhs and rhs bits: " << endl;
    cin >> lhsbits >> rhsbits;
    bismo_rt::InstrumentationData ret = run_benchmark_matmul(rows, cols, depth, lhsbits, rhsbits);
    printInstrumentationHeaders(ret);
    printInstrumentationData(ret);
  }
}

void benchmark_gemm_batch() {
  bool headers_printed = false;
  while(1) {
    int rows, depth, cols, lhsbits, rhsbits;
    cin >> rows;
    if(rows == 0) {
      return;
    }
    cin >> depth >> cols;
    cin >> lhsbits >> rhsbits;
    bismo_rt::InstrumentationData ret = run_benchmark_matmul(rows, cols, depth, lhsbits, rhsbits);
    if(!headers_printed) {
      printInstrumentationHeaders(ret);
      headers_printed = true;
    }
    printInstrumentationData(ret);
  }
}
