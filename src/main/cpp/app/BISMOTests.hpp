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

#include <cstring>
#include <string>
#include <iostream>
#include <vector>
#include <cmath>
using namespace std;
#include "BitSerialMatMulAccelDriver.hpp"
#include "BitSerialMatMulExecutor.hpp"
#include "gemmbitserial/test/testhelpers.hpp"

using namespace gemmbitserial;

void generateRandMatrixInt(size_t bits, size_t rows, size_t cols, int32_t** ret) {
  int32_t minVal = 0;
  int32_t maxVal = (1 << bits);
  for(size_t i = 0; i < rows; i++) {
  	for(size_t j = 0; j < cols; j++){
	    ret[i][j] = (int32_t) (rand() % maxVal);
  	}
  }
}


  //helper funtction to quantize a given matrix with a given matrix of thresholds
void quantizeMatrix(int32_t * a, int32_t ** b, size_t arows, size_t acols, size_t bcols, int32_t ** r ){
    for (int i = 0; i < arows; i++)
      for (int j = 0; j < acols; j++)
        for (int k = 0; k < bcols; k++)
        {
          if(a[i * acols + j] > b[i][k]){
            r[i][j]++;
          }
        }
  }


    //helper funtction to quantize a given matrix with a given matrix of thresholds
void quantizeMatrix2(int32_t * a, int32_t * b, size_t arows, size_t acols, size_t brows, size_t bcols, int32_t * r, size_t offset ){
    for (int i = 0; i < arows; i++)
      for (int j = 0; j < acols; j++)
        for (int k = 0; k < bcols; k++)
          //for (int l = 0; l < bcols; l++)
          {
            if(a[((i+offset) * acols) + j] > b[i * bcols + k] ){
              r[((i+offset) * acols) + j]++;
            }
          }
  }


void printmatrixInt(int32_t ** mat, int rows, int cols) {
  for(int i = 0; i < rows; i++) {
    for(int j = 0; j < cols; j++) {
      std::cout << (int) mat[i][j] << " ";
    }
    std::cout << std::endl;
  }
  std::cout << std::endl;
}


 
// BISMO top-level tests

bool test(
  string testName,
  WrapperRegDriver * platform, BitSerialMatMulAccelDriver * acc,
  size_t nrows_lhs, size_t nrows_rhs, size_t ncols, size_t nbits_lhs = 1,
  size_t nbits_rhs = 1, bool sgn_lhs = false, bool sgn_rhs = false,
  size_t ncols_ths= 15, size_t nbits_ths = 1, bool sgn_ths = false
) {
  //This is for tiling
  size_t ths_rows = nrows_lhs * (nrows_lhs / nrows_rhs);

  uint8_t * lhs = new uint8_t[nrows_lhs * ncols];
  uint8_t * rhs = new uint8_t[nrows_rhs * ncols];

  int32_t * ths_bs = new int32_t[nrows_lhs * ncols_ths];
  int32_t * qres_bs = new int32_t[nrows_rhs * nrows_lhs];

  for (int i = 0; i < nrows_rhs; i++)
    for (int j = 0; j < nrows_lhs; j++)
    {
      qres_bs[i* nrows_lhs + j] = 0;
    }

  // int32_t ** ths = new int32_t*[ths_rows];
  // int32_t ** qres = new int32_t*[ths_rows];
  // //Allocation of different matrix
  // for (int i = 0; i < ths_rows; ++i)
  // {
  // 	ths[i] = new int32_t[ncols_ths];
  //   qres[i] = new int32_t[nrows_lhs];
  // }
  // //Init quantized res matrix
  // for (int i = 0; i < ths_rows; i++)
  //   for (int j = 0; j < nrows_lhs; j++)
  //   {
  //     qres[i][j] = 0;
  //   }
  generateRandomVector(nbits_lhs, nrows_lhs*ncols, lhs);
  generateRandomVector(nbits_rhs, nrows_rhs*ncols, rhs);
  generateRandomVector(4, nrows_lhs * ncols_ths, ths_bs);
  printmatrix(ths_bs, nrows_lhs, ncols_ths );

/**************************** THS transfer ****************************/
  
  // generateRandMatrixInt(4, ths_rows, ncols_ths, ths);
  // printmatrixInt(ths, ths_rows, ncols_ths  );

  //TODO multi tile ths?
  for (int i = 0; i < ths_rows; i++)
  {
  	//TODO random vector for more than 8 bits
  	//generateRandMatrixInt(4, ncols_ths, &(ths[i]);
  	//cout << "Vector " << i << endl;
  	for(int j = 0; j < ncols_ths; j++ ){
  		//cout <<" Elem" << *(*(ths + i)+j) << " " <<  i << ", "<< j << ";";

      /******************************************************************/
      //TODO loading on different address depending on the rolling factor
      /******************************************************************/
  		acc->thsSingleTransfer(&ths_bs[i * ncols_ths + j], 0, i, j);
  	}
  	cout << endl;
  }
/**************************** END ****************************/

  GEMMContext ctx = acc->allocGEMMContext(
    nrows_lhs, ncols, nrows_rhs, nbits_lhs, nbits_rhs, sgn_lhs, sgn_rhs
  );
  ctx.lhs.importRegular(lhs);
  ctx.rhs.importRegular(rhs);
  gemmBitSerial(ctx);
  int32_t * accel_res = new int32_t[nrows_lhs*nrows_rhs];

  BitSerialMatMulExecutor * runner = new BitSerialMatMulExecutor(
    ctx, acc, platform
  );
  runner->setLHS(ctx.lhs);
  runner->setRHS(ctx.rhs);
  runner->run();
  runner->getRes(accel_res);


  // quantizeMatrix(ctx.res, ths, ths_rows, nrows_lhs, ncols_ths, qres);
  //TODO: Tiling properly
  quantizeMatrix2(ctx.res, ths_bs, nrows_lhs, nrows_lhs, nrows_lhs, ncols_ths, qres_bs, 0);
  quantizeMatrix2(ctx.res, ths_bs, nrows_lhs, nrows_lhs, nrows_lhs, ncols_ths, qres_bs, nrows_lhs);


  int res = memcmp(ctx.res, accel_res, nrows_lhs*nrows_rhs*sizeof(ResultType));
  //TODO final verification miss
  int resq = memcmp(qres_bs,accel_res,nrows_lhs*nrows_rhs*sizeof(ResultType));
  cout << "Quantization comparison result: " << resq << endl;

  if(res == 0) {
    cout << "Test succeeded (" << testName << ")" << endl;
    runner->printPerfSummary();
    runner->printPerfDetails();
  } else {
    cout << "Test failed (" << testName << ")" << endl;
    cout << "Expected: " << endl;
    printmatrix(ctx.res, nrows_rhs, nrows_lhs);
    cout << "Produced: " << endl;
    printmatrix(accel_res, nrows_rhs, nrows_lhs);
    cout << "Quantized: " << endl;
    // printmatrixInt(qres, nrows_rhs, nrows_lhs);
    printmatrix(qres_bs, nrows_rhs, nrows_lhs);

  }
  

  printf("Start deleting :)\n");

  delete runner;
  printf("Deleted runner \n");

  delete [] lhs;
  printf("Deleted lhs\n");

  delete [] rhs;
  // for (int i = 0; i < nrows_lhs; ++i)
  // {
  // 	delete[] ths[i];
  //   delete[] qres[i];
  // }
  // delete [] ths;
  // delete [] qres;
  printf("Deleted rhs\n");

  delete [] qres_bs;
  printf("Deleted qres\n");

  delete [] ths_bs;
  printf("Deleted ths\n");

  delete [] accel_res;
  printf("Deleted accel res\n");

  return res == 0;
}

bool test_binary_onchip_onetile(
  WrapperRegDriver * platform, BitSerialMatMulAccelDriver * acc
) {
  bool all_OK = true;
  vector<size_t> cols_div_factor {2, 4, 8};
  for(auto & col_div : cols_div_factor) {
    all_OK &= test(
      "binary_onchip_onetile_coldiv" + to_string(col_div), platform, acc,
      acc->hwcfg().dpaDimLHS, acc->hwcfg().dpaDimRHS,
      acc->hwcfg().dpaDimCommon * acc->hwcfg().lhsEntriesPerMem / col_div,
      1,1,false,false, std::pow(2,acc->hwcfg().maxQuantDim)-1, acc->hwcfg().accWidth, false
    );
  }

  return all_OK;
}

bool test_binary_size_independent(
  WrapperRegDriver * platform, BitSerialMatMulAccelDriver * acc
) {
  bool all_OK = true;
  all_OK &= test(
    "binary_size_independent_",
    platform, acc,
    17, 7, 11,
    1,1,false,false, std::pow(2,acc->hwcfg().maxQuantDim)-1, acc->hwcfg().accWidth, false
  );

  return all_OK;
}

bool test_binary_onchip_multitile(
  WrapperRegDriver * platform, BitSerialMatMulAccelDriver * acc
) {
  bool all_OK = true;
  vector<size_t> stripes {2, /*3,*/ 4};
  for(auto & lhs_stripes : stripes) {
    for(auto & rhs_stripes : stripes) {
      size_t ncols = acc->hwcfg().dpaDimCommon*acc->hwcfg().lhsEntriesPerMem / (lhs_stripes*rhs_stripes);
      size_t nrows_lhs = lhs_stripes*acc->hwcfg().dpaDimLHS;
      size_t nrows_rhs = rhs_stripes*acc->hwcfg().dpaDimRHS;
      all_OK &= test(
        "binary_onchip_multitile_" +
        to_string(nrows_lhs) + "x" + to_string(ncols) + "x"+ to_string(nrows_rhs),
        platform, acc, nrows_lhs, nrows_rhs, ncols,
        1,1,false,false, std::pow(2,acc->hwcfg().maxQuantDim)-1, acc->hwcfg().accWidth, false
      );
    }
  }
  return all_OK;
}

bool test_binary_offchip_multitile(
  WrapperRegDriver * platform, BitSerialMatMulAccelDriver * acc
) {
  bool all_OK = true;
  vector<size_t> stripes {2, /*3,*/ 4};
  for(auto & lhs_stripes : stripes) {
    for(auto & rhs_stripes : stripes) {
      size_t ncols = acc->hwcfg().dpaDimCommon*acc->hwcfg().lhsEntriesPerMem;
      size_t nrows_lhs = lhs_stripes*acc->hwcfg().dpaDimLHS;
      size_t nrows_rhs = rhs_stripes*acc->hwcfg().dpaDimRHS;
      all_OK &= test(
        "binary_offchip_multitile_" +
        to_string(nrows_lhs) + "x" + to_string(ncols) + "x"+ to_string(nrows_rhs),
        platform, acc, nrows_lhs, nrows_rhs, ncols,
        1,1,false,false, std::pow(2,acc->hwcfg().maxQuantDim)-1, acc->hwcfg().accWidth, false
      );
    }
  }
  return all_OK;
}

bool test_binary_offchip_widerows_multitile(
  WrapperRegDriver * platform, BitSerialMatMulAccelDriver * acc
) {
  bool all_OK = true;
  vector<size_t> lr_stripes {1, 2, 4};
  vector<size_t> z_stripes {2, 4};
  for(auto & lhs_stripe : lr_stripes) {
    for(auto & rhs_stripe : lr_stripes) {
      for(auto & z_stripe : z_stripes) {
        size_t ncols = acc->hwcfg().dpaDimCommon*acc->hwcfg().lhsEntriesPerMem*z_stripe;
        size_t nrows_lhs = lhs_stripe*acc->hwcfg().dpaDimLHS;
        size_t nrows_rhs = rhs_stripe*acc->hwcfg().dpaDimRHS;
        all_OK &= test(
          "test_binary_offchip_widerows_multitile_" +
          to_string(nrows_lhs) + "x" + to_string(ncols) + "x"+ to_string(nrows_rhs),
          platform, acc, nrows_lhs, nrows_rhs, ncols,
          1,1,false,false, std::pow(2,acc->hwcfg().maxQuantDim)-1, acc->hwcfg().accWidth, false
        );
      }
    }
  }
  return all_OK;
}
