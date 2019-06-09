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

#include "bismo_rt_internal.hpp"
#include "bismo_rt_matrix.hpp"
#include "bismo_rt_shared_buffer.hpp"
#include "gemmbitserial/test/testhelpers.hpp"

namespace bismo_rt {

// run a simple self-test on the SharedBuffer functionality
bool selftest_shared_buffer() {
  string test_name = "selftest_shared_buffer";
  cout << "Starting test:" << test_name << endl;
  size_t n_elems = 10;
  SharedBuffer<int32_t> * buf = new SharedBuffer<int32_t>(
    10, platform, "test_buf", false, false
  );
  int32_t * host_buf = buf->hostbuf();
  for(size_t i = 0; i < n_elems; i++) {
    host_buf[i] = i+1;
  }
  bool all_ok = true;
  // expect host-accel contents to be initially different
  // TODO this assumes no coherency
  all_ok &= (buf->compare() == false);
  // after host-accel sync they should be the same again
  buf->host2accel();
  all_ok &= (buf->compare() == true);
  cout << "Test result = " << all_ok << endl;
  delete buf;
  return all_ok;
}

bool selftest_matrix() {
  bool all_ok = true;
  string test_name = "selftest_matrix";
  cout << "Starting test:" << test_name << endl;
  vector<size_t> dim {3, 4, 5};
  vector<MatrixType> mtype {matTypeLHS, matTypeRHS, matTypeRes};
  Matrix<uint8_t> * imat;
  Matrix<int32_t> * rmat;
  for(auto & nrows: dim) {
    for(auto & ncols: dim) {
      for(auto & mt: mtype) {
        if(mt == matTypeRes) {
          rmat = new Matrix<int32_t>(
            nrows, ncols, 1, true, true, mt
          );
          rmat->printSummary();
          int32_t * buf = rmat->hostbuf();
          int32_t elem_sum = 0;
          size_t n_zeros = 0;
          for(int i = 0; i < rmat->elems(); i++) {
            buf[i] = i+1;
          }
          rmat->host2accel();
          // check contents of padded host buf
          //gemmbitserial::printmatrix(mat->hostbuf(), mat->outer(), mat->inner());
          //gemmbitserial::printmatrix(mat->padded_hostbuf(), mat->outer_a(), mat->inner_a());
          for(size_t i = 0; i < rmat->elems_a(); i++) {
            n_zeros += (rmat->padded_hostbuf()[i] == 0 ? 1 : 0);
            elem_sum += rmat->padded_hostbuf()[i];
          }
          all_ok &= ( elem_sum == (rmat->elems() * (rmat->elems() + 1))/2 );
          all_ok &= ( n_zeros == rmat->elems_a() - rmat->elems() );
          rmat->accel2host();
          for(int i = 0; i < rmat->elems(); i++) {
            all_ok &= (buf[i] == i+1);
          }
          delete rmat;
        } else {
          imat = new Matrix<uint8_t>(
            nrows, ncols, 1, false, mt != matTypeLHS, mt
          );
          imat->printSummary();
          uint8_t * buf = imat->hostbuf();
          int32_t elem_sum = 0;
          size_t n_zeros = 0;
          for(int i = 0; i < imat->elems(); i++) {
            buf[i] = i+1;
          }
          imat->host2accel();
          // check contents of padded host buf
          //gemmbitserial::printmatrix(mat->hostbuf(), mat->outer(), mat->inner());
          //gemmbitserial::printmatrix(mat->padded_hostbuf(), mat->outer_a(), mat->inner_a());
          for(size_t i = 0; i < imat->elems_a(); i++) {
            n_zeros += (imat->padded_hostbuf()[i] == 0 ? 1 : 0);
            elem_sum += imat->padded_hostbuf()[i];
          }
          all_ok &= ( elem_sum == (imat->elems() * (imat->elems() + 1))/2 );
          all_ok &= ( n_zeros == imat->elems_a() - imat->elems() );
          imat->accel2host();
          for(int i = 0; i < imat->elems(); i++) {
            all_ok &= (buf[i] == i+1);
          }
          delete imat;
        }
      }
    }
  }
  cout << "Test result " << test_name << ":" << all_ok << endl;
  return all_ok;
}

}
