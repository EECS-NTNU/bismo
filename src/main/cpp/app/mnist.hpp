#include <cstring>
#include <string>
#include <iostream>
#include <vector>
#include <cmath>
using namespace std;
#include "mnist_first.h"
#include "utils.hpp"
#define ROWS 28
#define COLS 28
#define BITS 8
#define THS_NUMBER 1
#define max(x,y) (x > y ? x : y)
#define min(x,y) (x < y ? x : y)

using namespace gemmbitserial;

bool test_qnn_mnist_0(
  WrapperRegDriver * platform, BitSerialMatMulAccelDriver * acc,
  size_t nrows_lhs = ROWS, size_t nrows_rhs = ROWS, size_t ncols= COLS, size_t nbits_lhs = BITS,
  size_t nbits_rhs = BITS, bool sgn_lhs = false, bool sgn_rhs = false,
  size_t ncols_ths= THS_NUMBER, size_t nbits_ths = 1, bool sgn_ths = false
) 
{
 //This is for tiling
 size_t ths_rows = max(nrows_lhs,nrows_rhs);

 uint8_t * lhs = new uint8_t[nrows_lhs * ncols];
 uint8_t * rhs = new uint8_t[nrows_rhs * ncols];

 memcpy(lhs,image, nrows_lhs * ncols * sizeof(uint8_t));
 memcpy(rhs,identity, nrows_rhs * ncols * sizeof(uint8_t));


 int32_t * ths = new int32_t[ths_rows * ncols_ths];
 memcpy(ths, th0, ths_rows *  ncols_ths * sizeof(int32_t));


 uint8_t * rhs_transposed = new uint8_t[ncols * nrows_rhs];
 uint8_t * lhs_transposed = new uint8_t[ncols * nrows_lhs];
 transpose <uint8_t>(rhs,nrows_rhs,ncols,rhs_transposed,ncols,nrows_rhs);
 std::cout << "RHS matrix transposed" << endl;
 printmatrix(rhs_transposed, ncols, nrows_rhs);
 std::cout << "RHS matrix" << endl;
 printmatrix(rhs,nrows_rhs,ncols);


 transpose <uint8_t>(lhs,nrows_lhs,ncols,lhs_transposed,ncols,nrows_lhs);
 std::cout << "LHS matrix transposed" << endl;
 printmatrix(lhs_transposed, ncols, nrows_lhs);
 std::cout << "LHS matrix" << endl;
 printmatrix(lhs,nrows_lhs,ncols);

 uint8_t * prequantmat = new uint8_t[nrows_rhs * nrows_lhs];
 
 int32_t * results_nt = new int32_t[nrows_rhs * nrows_lhs];
 int32_t * results_t = new int32_t[nrows_rhs * nrows_lhs];


 int32_t * qres_bs = new int32_t[nrows_rhs * nrows_lhs];

  for (int i = 0; i < nrows_rhs; i++)
    for (int j = 0; j < nrows_lhs; j++)
    {
      prequantmat[i* nrows_lhs + j] = 0;
      qres_bs[i* nrows_lhs + j] = 0;

    }

 dotProduct(lhs, rhs, nrows_lhs, ncols, nrows_rhs, ncols, results_nt);

 //quantize that matrix for just one threshold
 // quantizeMatrix(prequantmat, ths, nrows_rhs, nrows_lhs, 1, 1, qres_bs, 0);
 printmatrix(results_nt, nrows_rhs, nrows_lhs);
 printmatrix(qres_bs, nrows_rhs, nrows_lhs);
//This context has a different alignment!!!


 //Create a gemm bit serial context for golden results

 
  GEMMContext ctx = acc->allocGEMMContext(
    nrows_lhs, ncols, nrows_rhs, nbits_lhs, nbits_rhs, sgn_lhs, sgn_rhs
  );
  ctx.lhs.importRegular(lhs);
  ctx.rhs.importRegular(rhs);
  // ctx.lhs.printHex();
  // ctx.rhs.printHex();
  gemmBitSerial(ctx);
  int res = memcmp(ctx.res, prequantmat, nrows_lhs*nrows_rhs*sizeof(ResultType));
  cout << "result" << res << endl;
  printmatrix(ctx.res, nrows_lhs, nrows_rhs);


  ctx.lhs.importRegular(rhs);
  ctx.rhs.importRegular(lhs_transposed);
  // ctx.lhs.printHex();
  // ctx.rhs.printHex();
  gemmBitSerial(ctx);
  res = memcmp(ctx.res, prequantmat, nrows_lhs*nrows_rhs*sizeof(ResultType));
  cout << "result" << res << endl;
  printmatrix(ctx.res, nrows_rhs, nrows_lhs);


  delete [] prequantmat;
  delete [] results_nt;
  delete [] qres_bs;

}

/*
  int32_t * accel_res = new int32_t[nrows_lhs*nrows_rhs];
  
//Context to pass to the runner to execute
  //Create a gemm bit serial context for hw
  GEMMContext ctx_bs = acc->allocGEMMContext(
    nrows_lhs, ncols, nrows_rhs, 1, 1, sgn_lhs, sgn_rhs
  );

  BitSerialMatMulExecutor * runner = new BitSerialMatMulExecutor(
    ctx, acc, platform
  );

BitSerialMatrix bsm = BitSerialMatrix::alloc(
          8, 1, 256, false, 1, 64
        );




 int32_t dpa_lhs = acc->getdpaDimLHS();

/**************************** THS transfer ****************************
  
  int32_t addr_offset = 0;
  cout << "Thr dims " << ths_rows << ", " << ncols_ths << endl;
  for (int i = 0; i < ths_rows; i++)
  {
  	for(int j = 0; j < ncols_ths; j++ ){
  		acc->thsSingleTransfer(&ths[i * ncols_ths + j], addr_offset , i, j);
  	}
      if((i+1)%dpa_lhs == 0){
        addr_offset ++;
      }
  }
/**************************** END ****************************/

/*************************** P2S ****************************
  size_t nbytes_bitser_lhs = (nbits_lhs * nrows_lhs * ncols) / 8;
  size_t nbytes_bitser_rhs = (nbits_rhs * nrows_rhs * ncols) / 8;

  void * hw_src = runner->setP2S(1,nrows_lhs * ncols, lhs);
  void * hw_dst = runner->setP2SRes(nbits_lhs, 1, nrows_lhs  * ncols);
  uint32_t cycles = acc->p2s_exec_and_wait();
  uint8_t * hw_res_lhs = new uint8_t[nbytes_bitser_lhs];
  platform->copyBufferAccelToHost(hw_dst, hw_res_lhs, nbytes_bitser_lhs);
  int memcmpres = memcmp(hw_res_lhs, ctx.lhs.data, nbytes_bitser_lhs);
  cout << "Serialize first took " << cycles << " with result =" << memcmpres << endl;
  ctx.lhs.printHex();
  memcpy(bsm.data, hw_res_lhs, nbytes_bitser_lhs);
  bsm.printHex();
  memcmpres = memcmp(hw_res_lhs, bsm.data, nbytes_bitser_lhs);
  cout << "Serialize first with different allocation with result =" << memcmpres << endl;
  printmatrix(hw_res_lhs, nrows_lhs, ncols);



  //second one
  hw_src = runner->setP2S(nrows_rhs, ncols, rhs);
  hw_dst = runner->setP2SRes(nbits_rhs, nrows_rhs, ncols);
  cycles = acc->p2s_exec_and_wait();
  uint8_t * hw_res_rhs = new uint8_t[nbytes_bitser_rhs];
  platform->copyBufferAccelToHost(hw_dst, hw_res_rhs, nbytes_bitser_rhs);
  memcmpres = memcmp(hw_res_rhs, ctx.rhs.data, nbytes_bitser_rhs);
  cout << "Serialize second took " << cycles << " with result =" << memcmpres << endl;
  ctx.rhs.printHex();
  memcpy(bsm.data, hw_res_rhs, nbytes_bitser_rhs);
  bsm.printHex();
  printmatrix(hw_res_rhs, nrows_rhs, ncols);
  runner->setLHSRHSSerial(hw_res_lhs, hw_res_rhs);

  /**************************** END ****************************
  ctx_bs.lhs.importRegular(hw_res_lhs);
  ctx_bs.rhs.importRegular(hw_res_rhs);
  gemmBitSerial(ctx_bs);
  // runner->setLHS(ctx.lhs);
  // runner->setRHS(ctx.rhs);
  runner->run();
  runner->getRes(accel_res);

  //TODO: Tiling properly
  cout << "Dimensions " << nrows_lhs << ", " << nrows_rhs << endl;
  quantizeMatrix(ctx.res, ths, nrows_rhs, nrows_lhs, 1, 1, qres_bs, 0);

  printmatrix(ths, ths_rows, ncols_ths );
  int res = memcmp(ctx.res, accel_res, nrows_lhs*nrows_rhs*sizeof(ResultType));
  int res_2 = memcmp(ctx_bs.res, accel_res, nrows_lhs*nrows_rhs*sizeof(ResultType));
  cout << "different allocation result" << res_2 << endl;
  //TODO final verification miss
  int resq = memcmp(qres_bs,accel_res,nrows_lhs*nrows_rhs*sizeof(ResultType));
  cout << "Quantization comparison result: " << resq << endl;

  if(res == 0) {
    cout << "Test succeeded (" << "Mnist first layer" << ")" << endl;
    runner->printPerfSummary();
    runner->printPerfDetails();
  } else {
    cout << "Test failed (" << "Mnist first layer" << ")" << endl;
    cout << "Expected: " << endl;
    printmatrix(ctx.res, nrows_rhs, nrows_lhs);
    cout << "Produced: " << endl;
    printmatrix(accel_res, nrows_rhs, nrows_lhs);
    cout << "Quantized: " << endl;
    // printmatrixInt(qres, nrows_rhs, nrows_lhs);
    printmatrix(qres_bs, nrows_rhs, nrows_lhs);

  }
 

  printf("Start deleting :)\n");

   delete [] lhs;
   printf("Deleted lhs\n");

   delete [] rhs;
    //delete
   // delete [] hw_res_lhs;
  //   //delete
  // delete [] hw_res_rhs;
  // printf("Deleted rhs\n");

  delete [] qres_bs;
  printf("Deleted qres\n");

  delete [] ths;
  printf("Deleted ths\n");

  delete [] accel_res;
  printf("Deleted accel res\n");
  runner->cleanAccelBuff();
  // // Deletion of this object cause core dumps
  // delete runner;
  // printf("Deleted runner \n");
  
  return 1;//resq == 0;;
}
*/