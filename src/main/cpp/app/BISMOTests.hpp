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
using namespace std;
#include "BitSerialMatMulAccelDriver.hpp"
#include "BitSerialMatMulExecutor.hpp"
#include "gemmbitserial/test/testhelpers.hpp"

using namespace gemmbitserial;

// BISMO top-level tests

bool test(
  string testName,
  WrapperRegDriver * platform, BitSerialMatMulAccelDriver * acc,
  size_t nrows_lhs, size_t nrows_rhs, size_t ncols, size_t nbits_lhs = 1,
  size_t nbits_rhs = 1, bool sgn_lhs = false, bool sgn_rhs = false,
  size_t ncols_ths, size_t nbits_ths = 1, bool sgn_ths = false
) {
  uint8_t * lhs = new uint8_t[nrows_lhs * ncols];
  uint8_t * rhs = new uint8_t[nrows_rhs * ncols];
  int32_t * ths = new int32_t[nrows_lhs][ncols_ths];

  generateRandomVector(nbits_lhs, nrows_lhs*ncols, lhs);
  generateRandomVector(nbits_rhs, nrows_rhs*ncols, rhs);

/**************************** THS transfer ****************************/
  //TODO multi tile ths?
  for (int i = 0; i < nrows_lhs; i++)
  {
  	generateRandomVector(nbits_ths, ncols_ths, ths[i]);
  	cout << "Vector " << i;
  	for(int j = 0; j < ncols_ths; j++ ){
  		cout <<"Elems" << ths[i][j] << ", ";
  		thsSingleTransfer(&ths[i][j], 0, i, j);
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

  int res = memcmp(ctx.res, accel_res, nrows_lhs*nrows_rhs*sizeof(ResultType));

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
  }

  delete runner;

  delete [] lhs;
  delete [] rhs;
  delete [] ths;
  delete [] accel_res;

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
      1,1,false,false, std::pow(2,acc->hwcfg.maxQuantDim)-1, acc->hwcfg().accWidth, false
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
    1,1,false,false, std::pow(2,acc->hwcfg.maxQuantDim)-1, acc->hwcfg().accWidth, false
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
        1,1,false,false, std::pow(2,acc->hwcfg.maxQuantDim)-1, acc->hwcfg().accWidth, false
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
        1,1,false,false, std::pow(2,acc->hwcfg.maxQuantDim)-1, acc->hwcfg().accWidth, false
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
          1,1,false,false, std::pow(2,acc->hwcfg.maxQuantDim)-1, acc->hwcfg().accWidth, false
        );
      }
    }
  }
  return all_OK;
}
