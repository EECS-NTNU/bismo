// Author:  Davide Conficconi
// Date: 19/10/2018
// Revision: 0


#include <cassert>
#include <iostream>
using namespace std;
#include "platform.h"
#include "EmuTestP2SAccel.hpp"

// Cosim test for basic FetchStage behavior

// number of entries in each BRAM
#define BRAM_ENTRIES  (1 << 10)
// number of BRAMs
#define BRAM_COUNT    4

WrapperRegDriver * p;
EmuTestP2SAccel * dut;

//void bram_write(int sel, int addr, uint64_t data) {
//  assert(sel >= 0 && sel < BRAM_COUNT);
//  assert(addr >= 0 && addr < BRAM_ENTRIES);
//  dut->set_bram_sel(sel);
//  dut->set_bram_req_addr(addr);
//  dut->set_bram_req_writeData(data);
//  dut->set_bram_req_writeEn(1);
//  dut->set_bram_req_writeEn(0);
//}
//
//uint64_t bram_read(int sel, int addr) {
//  assert(sel >= 0 && sel < BRAM_COUNT);
//  assert(addr >= 0 && addr < BRAM_ENTRIES);
//  dut->set_bram_sel(sel);
//  dut->set_bram_req_addr(addr);
//  return dut->get_bram_rsp();
//}

void exec_and_wait() {
  dut->set_start(1);
  while(dut->get_done() != 1);
  dut->set_start(0);
}


void set_up_transfer_singletarget(
  void * accel_buf, uint32_t nbytes, void * accel_buf_dst
  /*uint32_t bram_sel, uint32_t bram_base, uint32_t bram_nwrites*/) {


 /* assert(bram_nwrites == nbytes/8);
  assert(bram_sel >= 0 && bram_sel < BRAM_COUNT);
  assert(bram_base >= 0 && ((bram_base + bram_nwrites) < BRAM_ENTRIES));
  dut->set_csr_bram_id_start(bram_sel);
  dut->set_csr_bram_id_range(bram_sel);
  dut->set_csr_bram_addr_base(bram_base);
  dut->set_csr_tiles_per_row(bram_nwrites);*/

  //dut->set_csr_dram_block_size_bytes(nbytes);
  dut->set_csr_dramBaseAddrSrc((AccelDblReg) accel_buf);
  cout << "[SW] DRAM Base Addr Src" << accel_buf << endl;
  dut->set_csr_dramBaseAddrDst((AccelDblReg) accel_buf_dst );
  dut->set_csr_matrixRows(8);
  dut->set_csr_matrixCols(8);
  dut->set_csr_actualInBw(0xFF);
  dut->set_csr_waitCompleteBytes(nbytes);


  dut->set_inDma_outer_step(0);
  dut->set_inDma_outer_count(1);
  dut->set_inDma_inner_step(0);
  dut->set_inDma_inner_count(1);


  dut->set_outDma_outer_step(0);
  dut->set_outDma_outer_count(1);
  dut->set_outDma_inner_step(0);
  dut->set_outDma_inner_count(1);
  
}/*

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
}*/

int main()
{
  bool all_OK = true;
  p = initPlatform();
  dut = new EmuTestP2SAccel(p);



  // copy a block of data to a single target BRAM
  // TODO set up mask here to exclude other channels
  bool singletarget_dram_OK = true;
  size_t nwords = 8;
  size_t nbytes = nwords * sizeof(uint64_t);
  uint32_t target_bram = 1;
  uint64_t * hostbuf = new uint64_t[nwords];
  uint64_t * hostbuf2  = new uint64_t[nwords];
  for(int i = 0; i < nwords; i++) {
    hostbuf[i] = i+1;
    //printf("%d\n",hostbuf[i] );
  }
  void * accelbuf = p->allocAccelBuffer(nbytes);
  void * accelbuf2 = p->allocAccelBuffer(nbytes);

  p->copyBufferHostToAccel(hostbuf, accelbuf, nbytes);
  //cout << "[SW] Copying data to the accelerator" << endl;
  set_up_transfer_singletarget(accelbuf, nbytes, accelbuf2);//target_bram, 0, nwords);
  //cout << "[SW] Set up all the params " << endl;
  exec_and_wait();
  //cout << "[SW] Done :) "<<endl; 

  p->copyBufferAccelToHost(accelbuf2, hostbuf2, nbytes );
  cout << "Result of Accel: " << endl;
  for (int i = 0; i < nwords; i++)
  {
    cout << "Pos:" << i << "Elem: " << hostbuf2[i] << endl;
  }
  //singletarget_dram_OK &= (memcmp_from_bram(target_bram, 0, nbytes, hostbuf) == 0);
  //cout << "DRAM to single-target test passed? " << singletarget_dram_OK << endl;
  all_OK &= singletarget_dram_OK;

  delete [] hostbuf;
  p->deallocAccelBuffer(accelbuf);
/*
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
*/

  delete dut;
  deinitPlatform(p);

  return all_OK ? 0 : -1;
}
