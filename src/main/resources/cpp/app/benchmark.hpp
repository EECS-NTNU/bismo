// Copyright (c) 2019 Xilinx
// Author: Yaman Umuroglu
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
// * Neither the name of [project] nor the names of its
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

using namespace std;
#include <iostream>
using namespace std;
#include "gemmbitserial/test/testhelpers.hpp"
#include "bismo_inference.hpp"

void run_benchmark_matmul(
  size_t nrows_lhs, size_t nrows_rhs, size_t ncols, size_t nbits_lhs,
  size_t nbits_rhs
) {
  uint8_t * lhs = new uint8_t[nrows_lhs * ncols];
  uint8_t * rhs = new uint8_t[nrows_rhs * ncols];
  gemmbitserial::generateRandomVector(nbits_lhs, nrows_lhs*ncols, lhs);
  gemmbitserial::generateRandomVector(nbits_rhs, nrows_rhs*ncols, rhs);
  bismo_inference::MatMulLayerDescriptor dscr;
  dscr.wbits = nbits_lhs;
  dscr.ibits = nbits_rhs;
  dscr.wsigned = false;
  dscr.isigned = false;
  dscr.M = nrows_lhs;
  dscr.K = ncols;
  dscr.N = nrows_rhs;
  bismo_inference::init();
  bismo_inference::LayerHandle id = bismo_inference::initMatMulLayer(dscr, lhs);
  int32_t * accel_res = new int32_t[nrows_lhs*nrows_rhs];
  bismo_inference::execMatMulLayer(id, rhs, accel_res);
  bismo_inference::deinit();

  delete [] lhs;
  delete [] rhs;
  delete [] accel_res;
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
    run_benchmark_matmul(rows, cols, depth, lhsbits, rhsbits);
  }
}
