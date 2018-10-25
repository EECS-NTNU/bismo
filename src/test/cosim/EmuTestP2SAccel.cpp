// Author:  Davide Conficconi
// Date: 19/10/2018
// Revision: 0


#include <cassert>
#include <iostream>
using namespace std;
#include "platform.h"
#include "EmuTestP2SAccel.hpp"
/***************************************************************/
#include "gemmbitserial/gemmbitserial.hpp"
#include "gemmbitserial/test/testhelpers.hpp"

#define FETCH_ADDRALIGN   8
#define FETCH_SIZEALIGN   8

#define max(x,y) (x > y ? x : y)
#define FETCH_ALIGN       max(FETCH_ADDRALIGN, FETCH_SIZEALIGN)
/***************************************************************/
typedef uint64_t PackedBitGroupType;
WrapperRegDriver * p;
EmuTestP2SAccel * dut;


  // allocate a GEMMContext compliant with the accelerator size
gemmbitserial::GEMMContext allocGEMMContext(
    uint64_t lhsRows, uint64_t depth, uint64_t rhsRows,
    uint64_t lhsBits, uint64_t rhsBits,
    bool lhsSigned, bool rhsSigned
  ) {
    const uint64_t regblock_lhs = 8;
    const uint64_t regblock_d = FETCH_ALIGN / sizeof(PackedBitGroupType);
    const uint64_t regblock_rhs = 8;
    const uint64_t cacheBits = 1;

    return gemmbitserial::allocGEMMContext_base(
      lhsRows, depth, rhsRows, lhsBits, rhsBits, lhsSigned, rhsSigned,
      regblock_lhs, regblock_d, regblock_rhs, cacheBits
    );
  }


void exec_and_wait() {
  dut->set_start(1);
  while(dut->get_done() != 1);
  dut->set_start(0);
}


void set_up_transfer_singletarget(
  void * accel_buf, uint32_t nbytes, void * accel_buf_dst) {


  dut->set_csr_dramBaseAddrSrc((AccelDblReg) accel_buf);
  cout << "[SW] DRAM Base Addr Src" << accel_buf << endl;
  dut->set_csr_dramBaseAddrDst((AccelDblReg) accel_buf_dst );
  dut->set_csr_matrixRows(1);
  dut->set_csr_matrixCols(8);
  dut->set_csr_actualInBw(0xFF);
  dut->set_csr_waitCompleteBytes(nbytes);


  dut->set_inDma_outer_step(8);
  dut->set_inDma_outer_count(1);
  dut->set_inDma_inner_step(8);
  dut->set_inDma_inner_count(8);


  dut->set_outDma_outer_step(8);
  dut->set_outDma_outer_count(1);
  dut->set_outDma_inner_step(8);
  dut->set_outDma_inner_count(8);
  
}

// int memcmp_bs_dram(){

// }

int main()
{
  bool all_OK = true;
  p = initPlatform();
  dut = new EmuTestP2SAccel(p);



  // copy a block of data to a single target BRAM
  // TODO set up mask here to exclude other channels
  bool singletarget_dram_OK = true;
  size_t nwords = 8;
  size_t nbytes = nwords * nwords * sizeof(uint8_t);

  uint8_t * hostbuf = new uint8_t[nwords * nwords];
  uint8_t * hostbuf2  = new uint8_t[nwords * nwords];
  //uint8_t * gold = new uint8_t[nwords * nwords];
  uint64_t * hw_res = new uint64_t[nwords * nwords];
  uint64_t mask  = 0x00000000000000FF;


  //Try to reproduce the gemm contest to produce golden result
  gemmbitserial::GEMMContext ctx = allocGEMMContext(
    nwords, nwords, nwords, nwords, nwords, false, false );

  generateRandomVector(nwords, nwords*nwords, hostbuf);

  ctx.lhs.importRegular(hostbuf);
  
/*
  for(int i = 0; i < nwords; i++) {
    hostbuf[i] = i+1070;
    uint64_t sum = 0;
    for (int j = 0; j < nwords; j++)
    {
      sum += ((hostbuf[i] & (mask << (j*8)) )>> (j*8));
    }
    gold[i] = sum;
    cout << gold[i] << ", ";
  }
  cout << endl;*/
  void * accelbuf = p->allocAccelBuffer(nbytes);
  void * accelbuf2 = p->allocAccelBuffer(nbytes);

  p->copyBufferHostToAccel(hostbuf, accelbuf, nbytes);
  //cout << "[SW] Copying data to the accelerator" << endl;
  set_up_transfer_singletarget(accelbuf, nbytes, accelbuf2);
  //cout << "[SW] Set up all the params " << endl;
  exec_and_wait();
  //cout << "[SW] Done :) "<<endl; 

  p->copyBufferAccelToHost(accelbuf2, hostbuf2, nbytes );
  ctx.lhs.printHex();

  cout << "[SW] Result collection and comparison" << endl; 
  cout << "Result of Accel: " << endl;
  //cout << hostbuf2[0] << endl;
  for (int i = 0; i < nwords; i++){
    //cout << hostbuf2[i] << " " << endl;
    for (int j = 0; j < nwords; j++)
    {
      cout << /*"Pos:" <<i << "," <<  j  << ": " <<*/   std::hex <<  static_cast<int>((hostbuf2[i * nwords + j])) << std::dec << ", "; 
      //((hostbuf2[i] & (mask << (j*8)) )>> (j*8)) << "; ";
    }
    cout << endl;
  }


  //cout << "Try to cast in proper way :D " << endl;
  for (int i = 0; i < nwords; i++){
    for (int j = 0; j < nwords; j++)
    {
      hw_res[i * nwords + j] = hostbuf2[i * nwords + j];
      //cout << std::hex <<  hw_res[i * nwords + j] << std::dec << ", ";
    }
    //cout << endl;
  }

  int res =  memcmp(hw_res, ctx.lhs.data, nwords * nwords * sizeof(uint64_t));
  cout << "Comparison result:" << res << endl;
  //singletarget_dram_OK &= (memcmp_from_bram(target_bram, 0, nbytes, hostbuf) == 0);
  //cout << "DRAM to single-target test passed? " << singletarget_dram_OK << endl;
  all_OK &= singletarget_dram_OK;

  delete [] hostbuf;
  delete [] hostbuf2;
  p->deallocAccelBuffer(accelbuf);
  p->deallocAccelBuffer(accelbuf2);

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