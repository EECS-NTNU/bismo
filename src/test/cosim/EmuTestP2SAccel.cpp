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
#define MAX_ACCEL_BITWIDTH 8

/***************************************************************/
typedef uint64_t PackedBitGroupType;
WrapperRegDriver * p;
EmuTestP2SAccel * dut;

uint64_t rightRotate(uint64_t n, int32_t d){
  return (n >> d) | (n << (64 - d));
}

uint64_t leftRotate(uint64_t n, int32_t d){
  return (n << d) | (n >> (64 - d));
}


  // allocate a GEMMContext compliant with the accelerator size
gemmbitserial::GEMMContext allocGEMMContext(
    uint64_t lhsRows, uint64_t depth, uint64_t rhsRows,
    uint64_t lhsBits, uint64_t rhsBits,
    bool lhsSigned, bool rhsSigned
  ) {
    const uint64_t regblock_lhs = 1;
    const uint64_t regblock_d = FETCH_ALIGN / sizeof(PackedBitGroupType);
    const uint64_t regblock_rhs = 1;
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


void set_up_transfer_singletarget( void * accel_buf, uint32_t nbytes, void * accel_buf_dst, uint32_t rows,  uint32_t cols,  uint32_t bit_width) {


  dut->set_csr_dramBaseAddrSrc((AccelDblReg) accel_buf);
  cout << "[SW] DRAM Base Addr Src" << accel_buf << endl;
  dut->set_csr_dramBaseAddrDst((AccelDblReg) accel_buf_dst );
  dut->set_csr_matrixRows(rows);
  dut->set_csr_matrixCols(cols);
  dut->set_csr_actualInBw(bit_width);
  dut->set_csr_waitCompleteBytes(nbytes);

}

int main()
{
  bool all_OK = true;
  p = initPlatform();
  dut = new EmuTestP2SAccel(p);


  bool singletarget_dram_OK = true;
  int test_count = 2; //a number between 0 and Maximum accel-input bit-width - 1 
  int initVal = 1;
  for (int i = initVal; i < test_count; i++)
  {

    int32_t ncols = 80;//i * i * 8 + 8;
    int32_t nrows = 1;
    size_t nbytes = nrows * ncols * sizeof(uint8_t);
    uint8_t nbits = 3;//i + 1;

    uint32_t dramWordWidth = 64;
    uint32_t dramByteWidth = dramWordWidth / 8;

    int32_t mybitwidth = sizeof(uint8_t) /*+ i + 1;*/  * 8;
    int32_t colPerDramBWidth = max(ncols / dramWordWidth, dramByteWidth);
    //Preventing a zero count
    int32_t bscolNumber = max(ncols / dramWordWidth, 1);
    size_t bs_size_bytes = mybitwidth * nrows * colPerDramBWidth;


    uint8_t * hostbuf = new uint8_t[nrows * ncols];
    uint8_t * hostbuf2  = new uint8_t[bs_size_bytes];

    //Reproduce the gemm contest to produce golden result
    gemmbitserial::GEMMContext ctx = allocGEMMContext(
      nrows, ncols, nrows, nbits, nbits, false, false );

    generateRandomVector(nbits, ncols, hostbuf);
    printmatrix(hostbuf,nrows, ncols);
    ctx.lhs.importRegular(hostbuf);
    
    void * accelbuf = p->allocAccelBuffer(nbytes);
    void * accelbuf_prova = p->allocAccelBuffer(nbytes);
    void * accelbuf2 = p->allocAccelBuffer(bs_size_bytes);


    p->copyBufferHostToAccel(hostbuf, accelbuf, nbytes);
    set_up_transfer_singletarget(accelbuf, bs_size_bytes, accelbuf2, nrows, ncols, mybitwidth );
    exec_and_wait();



    p->copyBufferAccelToHost(accelbuf2, hostbuf2, bs_size_bytes);
    ctx.lhs.printHex();

    cout << "[SW] Result collection and comparison" << endl; 
   
    cout << "Result of Accel: " << endl;
    for (int i = 0; i < mybitwidth; i++){
      for (int j = 0; j < nrows; j++){
        for(int k = 0; k < bscolNumber; k++)
        {
          cout << "Index: " << (i * nrows * bscolNumber + j * bscolNumber + k) << ", ";
          cout << std::hex << ((uint64_t*)(hostbuf2))[i * nrows * bscolNumber + j * bscolNumber + k] << std::dec << ", "; 
        }
      }
      cout << endl; 
    }

    cout << endl;
    cout << std::hex << ((uint64_t*)(hostbuf2))[0] << endl;
    cout << std::hex << ((uint64_t*)(hostbuf2))[1] << endl;



    int res =  memcmp(hostbuf2, ctx.lhs.data, nbits * nrows * ncols / 8 );
    cout << "Comparison result:" << res << endl;
    //singletarget_dram_OK &= (memcmp_from_bram(target_bram, 0, nbytes, hostbuf) == 0);
    //cout << "DRAM to single-target test passed? " << singletarget_dram_OK << endl;
    all_OK &= !res;

    delete [] hostbuf;
    delete [] hostbuf2;
    p->deallocAccelBuffer(accelbuf);
    p->deallocAccelBuffer(accelbuf2);

  }

  delete dut;
  deinitPlatform(p);

  return all_OK ? 0 : -1;
}