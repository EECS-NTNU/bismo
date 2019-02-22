// Copyright (c) 2018 Norwegian University of Science and Technology (NTNU)
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

#include <cassert>
#include <iostream>
using namespace std;
#include "platform.h"
#include "EmuTestFetchStage.hpp"

// Cosim test for basic FetchStage behavior

// number of entries in each BRAM
#define BRAM_ENTRIES  (1 << 10)
// number of BRAMs
#define BRAM_COUNT    4

WrapperRegDriver * p;
EmuTestFetchStage * dut;

void bram_write(int sel, int addr, uint64_t data) {
  assert(sel >= 0 && sel < BRAM_COUNT);
  assert(addr >= 0 && addr < BRAM_ENTRIES);
  dut->set_bram_sel(sel);
  dut->set_bram_req_addr(addr);
  dut->set_bram_req_writeData(data);
  dut->set_bram_req_writeEn(1);
  dut->set_bram_req_writeEn(0);
}

uint64_t bram_read(int sel, int addr) {
  assert(sel >= 0 && sel < BRAM_COUNT);
  assert(addr >= 0 && addr < BRAM_ENTRIES);
  dut->set_bram_sel(sel);
  dut->set_bram_req_addr(addr);
  return dut->get_bram_rsp();
}

void exec_and_wait() {
  dut->set_start(1);
  while(dut->get_done() != 1);
  dut->set_start(0);
}

// TODO: this test needs to be updated to respect the fact that we only allow
// fetching for either all LHS or all RHS BRANs, not single ones

void set_up_transfer_singletarget(
  void * accel_buf, uint32_t nbytes,
  uint32_t bram_sel, uint32_t bram_base, uint32_t bram_nwrites) {
  assert(bram_nwrites == nbytes/8);
  assert(bram_sel >= 0 && bram_sel < BRAM_COUNT);
  assert(bram_base >= 0 && ((bram_base + bram_nwrites) < BRAM_ENTRIES));
  dut->set_csr_bram_id_start(bram_sel);
  dut->set_csr_bram_id_range(bram_sel);
  dut->set_csr_bram_addr_base(bram_base);
  dut->set_csr_tiles_per_row(bram_nwrites);

  dut->set_csr_dram_block_size_bytes(nbytes);
  dut->set_csr_dram_base((AccelDblReg) accel_buf);
  // this test uses a single block
  dut->set_csr_dram_block_offset_bytes(0);
  dut->set_csr_dram_block_count(1);
}

void set_up_transfer_multitarget(void * accel_buf, uint32_t nbytes,
  uint32_t bram_start, uint32_t bram_idrange, uint32_t bram_base,
  uint32_t writes_per_bram_turn
) {
  dut->set_csr_bram_id_start(bram_start);
  dut->set_csr_bram_id_range(bram_idrange);
  dut->set_csr_bram_addr_base(bram_base);
  dut->set_csr_tiles_per_row(writes_per_bram_turn);

  dut->set_csr_dram_block_size_bytes(nbytes);
  dut->set_csr_dram_base((AccelDblReg) accel_buf);
  // this test uses a single block
  dut->set_csr_dram_block_offset_bytes(0);
  dut->set_csr_dram_block_count(1);
}

void memcpy_from_bram(int bram_sel, int bram_base, int nbytes, void *dest) {
  assert(nbytes % 8 == 0);
  uint64_t * dest_8b = (uint64_t *) dest;
  for(int i = 0; i < nbytes/8; i++) {
    dest_8b[i] = bram_read(bram_sel, bram_base+i);
  }
}

int memcmp_from_bram(int bram_sel, int bram_base, int nbytes, void *cmp) {
  assert(nbytes % 8 == 0);
  uint64_t * dest_8b = (uint64_t *) cmp;
  int ret = 0;
  for(int i = 0; i < nbytes/8; i++) {
    if(dest_8b[i] != bram_read(bram_sel, bram_base+i)) {
      ret = -1;
      cout << "difference in memcmp_from_bram!" << endl;
      cout << "expected: " << dest_8b[i];
      cout << " found: " << bram_read(bram_sel, bram_base+i) << endl;
    }
  }
  return ret;
}

void zero_bram(int bram_sel) {
  for(int i = 0; i < BRAM_ENTRIES; i++) {
    bram_write(bram_sel, i, 0);
  }
}

int main()
{
  bool all_OK = true;
  p = initPlatform();
  dut = new EmuTestFetchStage(p);

  // check that the BRAM access muxes are working correctly
  bool bram_OK = true;
  bram_write(0, 10, 0xdeadbeef);
  bram_write(1, 20, 0xbeadfeed);
  bram_OK &= (bram_read(0, 10) == 0xdeadbeef);
  bram_OK &= (bram_read(1, 20) == 0xbeadfeed);
  cout << "BRAM simple access test passed? " << bram_OK << endl;
  all_OK &= bram_OK;

  // copy a block of data to a single target BRAM
  // TODO set up mask here to exclude other channels
  bool singletarget_dram_OK = true;
  size_t nwords = 8;
  size_t nbytes = nwords * sizeof(uint64_t);
  uint32_t target_bram = 1;
  uint64_t * hostbuf = new uint64_t[nwords];
  for(int i = 0; i < nwords; i++) {
    hostbuf[i] = i+1;
  }
  void * accelbuf = p->allocAccelBuffer(nbytes);
  p->copyBufferHostToAccel(hostbuf, accelbuf, nbytes);
  set_up_transfer_singletarget(accelbuf, nbytes, target_bram, 0, nwords);
  exec_and_wait();
  singletarget_dram_OK &= (memcmp_from_bram(target_bram, 0, nbytes, hostbuf) == 0);
  cout << "DRAM to single-target test passed? " << singletarget_dram_OK << endl;
  all_OK &= singletarget_dram_OK;

  delete [] hostbuf;
  p->deallocAccelBuffer(accelbuf);

  // copy data into all BRAMs (in turn) from a single DRAM transfer
  bool multitarget_dram_OK = true;
  size_t nwords_per_bram = 16;
  size_t nbytes_per_bram = nwords_per_bram * sizeof(uint64_t);
  nwords = nwords_per_bram * BRAM_COUNT;
  nbytes = nwords * sizeof(uint64_t);
  hostbuf = new uint64_t[nwords];
  for(int i = 0; i < nwords; i++) {
    hostbuf[i] = i+10;
  }
  accelbuf = p->allocAccelBuffer(nbytes);
  p->copyBufferHostToAccel(hostbuf, accelbuf, nbytes);
  set_up_transfer_multitarget(accelbuf, nbytes, 0, BRAM_COUNT-1, 0, nwords_per_bram);
  exec_and_wait();
  for(int i = 0; i < BRAM_COUNT; i++) {
    multitarget_dram_OK &= (memcmp_from_bram(i, 0, nbytes_per_bram, &hostbuf[i * nwords_per_bram]) == 0);
  }
  cout << "DRAM to multi-target test passed? " << multitarget_dram_OK << endl;
  all_OK &= multitarget_dram_OK;

  delete [] hostbuf;
  p->deallocAccelBuffer(accelbuf);


  delete dut;
  deinitPlatform(p);

  return all_OK ? 0 : -1;
}
