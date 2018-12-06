#include <cstring>
#include <string>
#include <iostream>
#include <vector>
#include <cmath>
using namespace std;
#include "mnist_first.h"
#define MATRIX_SIZE 28
#define BITS 8
#define max(x,y) (x > y ? x : y)
#define min(x,y) (x < y ? x : y)

using namespace gemmbitserial;




bool test_qnn_mnist_0(
  WrapperRegDriver * platform, BitSerialMatMulAccelDriver * acc,
  size_t nrows_lhs = MATRIX_SIZE, size_t nrows_rhs =MATRIX_SIZE, size_t ncols= MATRIX_SIZE, size_t nbits_lhs = 1,
  size_t nbits_rhs = 1, bool sgn_lhs = false, bool sgn_rhs = false,
  size_t ncols_ths= 1, size_t nbits_ths = 1, bool sgn_ths = false
) {
  //This is for tiling
  size_t ths_rows = max(MATRIX_SIZE,MATRIX_SIZE);

  uint8_t * lhs = new uint8_t[nrows_lhs * ncols];
  uint8_t * rhs = new uint8_t[nrows_rhs * ncols];

  memcpy(lhs,image, MATRIX_SIZE * MATRIX_SIZE * sizeof(uint8_t));
  memcpy(rhs,identity, MATRIX_SIZE * MATRIX_SIZE * sizeof(uint8_t));


  int32_t * ths_bs = new int32_t[ths_rows * ncols_ths];
  memcpy(ths_bs, th0, MATRIX_SIZE * sizeof(int32_t));

  int32_t * qres_bs = new int32_t[nrows_rhs * nrows_lhs];

  for (int i = 0; i < nrows_rhs; i++)
    for (int j = 0; j < nrows_lhs; j++)
    {
      qres_bs[i* nrows_lhs + j] = 0;
    }

  int32_t dpa_lhs = acc->getdpaDimLHS();

/**************************** THS transfer ****************************/
  

  //TODO multi tile ths?
  int32_t addr_offset = 0;
  cout << "Thr dims " << ths_rows << ", " << ncols_ths << endl;
  for (int i = 0; i < ths_rows; i++)
  {
  	//TODO random vector for more than 8 bits
  	// cout << "Vector " << i << endl;
  	for(int j = 0; j < ncols_ths; j++ ){
  		// cout <<" Elem" << ths_bs[i * ncols_ths + j] << " " <<  i << ", "<< j << ";";

      /******************************************************************/
      //TODO loading on different address depending on the rolling factor
      /******************************************************************/
  		acc->thsSingleTransfer(&ths_bs[i * ncols_ths + j], addr_offset , i, j);
  	}
      if((i+1)%dpa_lhs == 0){
        addr_offset ++;
      }
  	 // cout << endl;
  }
/**************************** END ****************************/

  GEMMContext ctx = acc->allocGEMMContext(
    nrows_lhs, ncols, nrows_rhs, nbits_lhs, nbits_rhs, sgn_lhs, sgn_rhs
  );
  ctx.lhs.importRegular(lhs);
  ctx.rhs.importRegular(rhs);
  ctx.lhs.printHex();
  ctx.rhs.printHex();
  gemmBitSerial(ctx);
  int32_t * accel_res = new int32_t[nrows_lhs*nrows_rhs];

  BitSerialMatMulExecutor * runner = new BitSerialMatMulExecutor(
    ctx, acc, platform
  );

  /**************************** P2S ****************************
  size_t nbytes_bitser_lhs = (nbits_lhs * nrows_lhs * ncols) / 8;
  size_t nbytes_bitser_rhs = (nbits_rhs * nrows_rhs * ncols) / 8;

  void * hw_src = runner->setP2S(nrows_lhs, ncols, lhs);
  void * hw_dst = runner->setP2SRes(nbits_lhs, nrows_lhs, ncols);
  uint32_t cycles = acc->p2s_exec_and_wait();
  uint8_t * hw_res_lhs = new uint8_t[nbytes_bitser_lhs];
  platform->copyBufferAccelToHost(hw_dst, hw_res_lhs, nbytes_bitser_lhs);
  int memcmpres = memcmp(hw_res_lhs, ctx.lhs.data, nbytes_bitser_lhs);
  cout << "Serialize first took " << cycles << " with result =" << memcmpres << endl;



  //second one
  hw_src = runner->setP2S(nrows_rhs, ncols, rhs);
  hw_dst = runner->setP2SRes(nbits_rhs, nrows_rhs, ncols);
  cycles = acc->p2s_exec_and_wait();
  uint8_t * hw_res_rhs = new uint8_t[nbytes_bitser_rhs];
  platform->copyBufferAccelToHost(hw_dst, hw_res_rhs, nbytes_bitser_rhs);
  memcmpres = memcmp(hw_res_rhs, ctx.rhs.data, nbytes_bitser_rhs);
  cout << "Serialize second took " << cycles << " with result =" << memcmpres << endl;

  // runner->setLHSRHSSerial(hw_res_lhs, hw_res_rhs);

  **************************** END ****************************/

  runner->setLHS(ctx.lhs);
  runner->setRHS(ctx.rhs);
  runner->run();
  runner->getRes(accel_res);

  //TODO: Tiling properly
  cout << "Dimensions " << nrows_lhs << ", " << nrows_rhs << endl;
  quantizeMatrix(ctx.res, ths_bs, nrows_rhs, nrows_lhs, ths_rows, ncols_ths, qres_bs, 0);

  printmatrix(ths_bs, ths_rows, ncols_ths );
  int res = memcmp(ctx.res, accel_res, nrows_lhs*nrows_rhs*sizeof(ResultType));
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

  delete [] ths_bs;
  printf("Deleted ths\n");

  delete [] accel_res;
  printf("Deleted accel res\n");
  runner->cleanAccelBuff();
  // // Deletion of this object cause core dumps
  // delete runner;
  // printf("Deleted runner \n");

  return resq == 0;;
}