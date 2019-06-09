// Copyright (c) 2018 Norwegian University of Science and Technology (NTNU)
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

#include <cassert>
#include <iostream>
using namespace std;
#include "platform.h"
#include "EmuTestResultStage.hpp"

// Cosim test for basic ResultStage behavior

#define DPA_LHS   2
#define DPA_RHS   2
typedef int32_t   ResultType;

WrapperRegDriver * p;
EmuTestResultStage * dut;

void exec_and_wait() {
  dut->set_start(1);
  while(dut->get_done() != 1);
  dut->set_start(0);
}

// get offset of element (x, y) in a 1D array[nx][ny]
int offset2D(int x, int y, int ny) {
  return x * ny + y;
}

void do_result_write(void * accel_buf, int ind_rhs, int ind_lhs, int num_rhs, int num_lhs) {
  size_t stride = num_lhs * sizeof(ResultType);
  uint64_t base = sizeof(int32_t) * offset2D(ind_rhs, ind_lhs, num_lhs) + (uint64_t) accel_buf;
  dut->set_csr_dram_base(base);
  dut->set_csr_dram_skip(stride);
  dut->set_csr_waitComplete(0);
  dut->set_csr_waitCompleteBytes(0);
  dut->set_csr_resmem_addr(0);
  exec_and_wait();
}

void do_result_waitcomplete(size_t nbytes) {
  dut->set_csr_waitComplete(1);
  dut->set_csr_waitCompleteBytes(nbytes);
  exec_and_wait();
}

void write_resmem(int lhs, int rhs, int val) {
  dut->set_accwr_lhs(lhs);
  dut->set_accwr_rhs(rhs);
  dut->set_accwr_data(val);
  dut->set_accwr_en(1);
  dut->set_accwr_en(0);
}

int main()
{
  bool all_OK = true;
  p = initPlatform();
  dut = new EmuTestResultStage(p);

  bool simple_res_OK = true;
  const size_t nrhs = DPA_RHS * 1, nlhs = DPA_LHS * 2;
  size_t nbytes = sizeof(ResultType) * nrhs * nlhs;
  uint32_t * hostbuf = new uint32_t[nrhs * nlhs];
  void * accelbuf = p->allocAccelBuffer(nbytes);

  // each tile is written to memory in column-major order, with appropriate
  // stride to account for the tiling. e.g the 2x2 DPA array:
  //   (<---rhs dim--->)       ^
  //          A B            lhs dim
  //          C D              v
  // turns into:
  // A, C, [stride], B, D
  // when written into memory, so the lhs entires remain contiguous, "rhs-major"

  //dut->set_acc_in_0_0(1);
  write_resmem(0, 0, 1);
  //dut->set_acc_in_1_0(2);
  write_resmem(1, 0, 2);
  //dut->set_acc_in_0_1(5);
  write_resmem(0, 1, 5);
  //dut->set_acc_in_1_1(6);
  write_resmem(1, 1, 6);
  do_result_write(accelbuf, 0 * DPA_RHS, 0 * DPA_LHS, nrhs, nlhs);

  //dut->set_acc_in_0_0(3);
  write_resmem(0, 0, 3);
  //dut->set_acc_in_1_0(4);
  write_resmem(1, 0, 4);
  //dut->set_acc_in_0_1(7);
  write_resmem(0, 1, 7);
  //dut->set_acc_in_1_1(8);
  write_resmem(1, 1, 8);
  do_result_write(accelbuf, 0 * DPA_RHS, 1 * DPA_LHS, nrhs, nlhs);

  // wait until all writes are completed
  do_result_waitcomplete(nbytes);

  p->copyBufferAccelToHost(accelbuf, hostbuf, nbytes);

  for(int i = 0; i < nrhs * nlhs; i++) {
    simple_res_OK &= (hostbuf[i] == i+1);
  }
  cout << "Simple result write test passed? " << simple_res_OK << endl;
  if(!simple_res_OK) {
    for(int i = 0; i < nrhs * nlhs; i++) {
      cout << "i[" << i << "] = " << hostbuf[i] << endl;
    }
  }
  all_OK &= simple_res_OK;

  delete [] hostbuf;

  delete dut;
  deinitPlatform(p);

  return all_OK ? 0 : -1;
}
