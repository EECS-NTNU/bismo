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
#define BASECOLUMNS 512 // Basic number of columns to tes, always multiple of the word
#define BASEROWS 8 // Basic number of rows to test
#define DRAMWORDBIT 64 //Memory word bits
#define INIT_TEST_VALUE 0
#define MAX_TEST_VALUE 7 // a number between 0 and maximum bit-width - 1

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
  for (int i = 0; i < 3000; i++);
  dut->set_start(1);
  dut->set_ackqueue_ready(false);
  while(dut->get_ackqueue_valid() != 1);
  dut->set_start(0);
  dut->set_ackqueue_ready(true);
  dut->set_ackqueue_ready(false);

}


void set_up_transfer_singletarget( void * accel_buf, uint32_t nbytes, void * accel_buf_dst, uint32_t rows,  uint32_t cols,  uint32_t bit_width) {

  
  dut->set_cmdqueue_bits_dramBaseAddrSrc((AccelDblReg) accel_buf);
  // cout << "[SW] DRAM Base Addr Src" << accel_buf << endl;
  dut->set_cmdqueue_bits_dramBaseAddrDst((AccelDblReg) accel_buf_dst );
  dut->set_cmdqueue_bits_matrixRows(rows);
  dut->set_cmdqueue_bits_matrixColsGroup(cols);
  dut->set_cmdqueue_bits_actualPrecision(bit_width);
  dut->set_cmdqueue_bits_waitCompleteBytes(nbytes);
  dut->set_cmdqueue_valid(true);
  dut->set_cmdqueue_valid(false);
}

int main()
{
  bool all_OK = true;
  p = initPlatform();
  dut = new EmuTestP2SAccel(p);


  bool singletarget_dram_OK = true;
  int test_count = MAX_TEST_VALUE; //a number between 0 and Maximum accel-input bit-width - 1 
  int initVal = INIT_TEST_VALUE;
  for (int i = initVal; i < test_count; i++)
  {

    int32_t ncols = BASECOLUMNS * (i + 1);
    int32_t nrows = BASEROWS * (i+1) ;
    size_t nbytes = nrows * ncols * sizeof(uint8_t);
    uint8_t nbits = 7;//1 * (i+1);

    uint32_t dramWordWidth = DRAMWORDBIT;
    uint32_t dramByteWidth = dramWordWidth / 8;

    int32_t mybitwidth = sizeof(uint8_t) * 8;
    int32_t container_szie = sizeof(uint8_t) * 8;
    int32_t colPerDramBWidth = ncols / dramWordWidth;
    //Preventing a zero count
    int32_t bscolNumber = ncols /dramWordWidth;
    size_t bs_size_bytes = mybitwidth * nrows * ncols/dramByteWidth;


    uint8_t * hostbuf = new uint8_t[nrows * ncols];
    uint8_t * hostbuf2  = new uint8_t[bs_size_bytes];

    //Reproduce the gemm contest to produce golden result
    gemmbitserial::GEMMContext ctx = allocGEMMContext(
      nrows, ncols, nrows, mybitwidth, mybitwidth, false, false );

    generateRandomVector(nbits, ncols, hostbuf);
    printmatrix(hostbuf,nrows, ncols);
    ctx.lhs.importRegular(hostbuf);
    
    void * accelbuf = p->allocAccelBuffer(nbytes);
    void * accelbuf2 = p->allocAccelBuffer(bs_size_bytes);


    p->copyBufferHostToAccel(hostbuf, accelbuf, nbytes);
    set_up_transfer_singletarget(accelbuf, bs_size_bytes, accelbuf2, nrows, colPerDramBWidth , mybitwidth );
    exec_and_wait();



    p->copyBufferAccelToHost(accelbuf2, hostbuf2, bs_size_bytes);
    ctx.lhs.printHex();

    cout << "[SW] Matrix rows: " << nrows << ", columns: " << ncols << ", bit-width: " << endl;
    cout << "[SW] Result collection and comparison" << endl; 
   
    cout << "Result of Accel: " << endl;
    for (int i = 0; i < mybitwidth; i++){
      for (int j = 0; j < nrows; j++){
        // for(int k = 0; k < bscolNumber; k++)
        {
          // cout << "Index: " << (i * nrows * bscolNumber + j * bscolNumber + k) << ", ";
          cout << "Index: " << (i * nrows + j) << ", ";
          // cout << std::hex << ((uint64_t*)(hostbuf2))[i * nrows * bscolNumber + j * bscolNumber + k] << std::dec << ", "; 
          cout << std::hex << ((uint64_t*)(hostbuf2))[i * nrows + j] << std::dec << ", "; 

        }
        // scout << endl;
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