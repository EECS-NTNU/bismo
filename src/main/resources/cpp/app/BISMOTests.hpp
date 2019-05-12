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
#include <algorithm>
#include <iostream>
#include <vector>
using namespace std;
#include "gemmbitserial/test/testhelpers.hpp"
#include "gemmbitserial/gemmbitserial.hpp"
#include "gemmbitserial/convbitserial.hpp"
#include "bismo_inference.hpp"

// BISMO top-level tests

bool test(
  string testName,
  size_t nrows_lhs, size_t nrows_rhs, size_t ncols, size_t nbits_lhs = 1,
  size_t nbits_rhs = 1, bool sgn_lhs = false, bool sgn_rhs = false
) {
  uint8_t * lhs = new uint8_t[nrows_lhs * ncols];
  uint8_t * rhs = new uint8_t[nrows_rhs * ncols];
  gemmbitserial::generateRandomVector(nbits_lhs, nrows_lhs*ncols, lhs);
  gemmbitserial::generateRandomVector(nbits_rhs, nrows_rhs*ncols, rhs);
  gemmbitserial::GEMMContext ctx = gemmbitserial::allocGEMMContext(
    nrows_lhs, ncols, nrows_rhs, nbits_lhs, nbits_rhs, sgn_lhs, sgn_rhs
  );
  cout << "Starting test: " << testName << endl;
  ctx.lhs.importRegular(lhs);
  ctx.rhs.importRegular(rhs);
  gemmbitserial::gemmBitSerial(ctx);
  cout << testName << ": " << nrows_lhs << "x" << ncols << "x" << nrows_rhs;
  cout << " " << nbits_lhs << "bx" << nbits_rhs << "b ";
  cout << "signed? " << sgn_lhs << " " << sgn_rhs << endl;

  bismo_inference::MatMulLayerDescriptor dscr;
  dscr.wbits = nbits_lhs;
  dscr.ibits = nbits_rhs;
  dscr.wsigned = sgn_lhs;
  dscr.isigned = sgn_rhs;
  dscr.M = nrows_lhs;
  dscr.K = ncols;
  dscr.N = nrows_rhs;
  bismo_inference::init();
  bismo_inference::LayerHandle id = bismo_inference::initMatMulLayer(dscr, lhs);
  int32_t * accel_res = new int32_t[nrows_lhs*nrows_rhs];
  bismo_inference::execMatMulLayer(id, rhs, accel_res);
  bismo_inference::deinitLayer(id);
  bismo_inference::deinit();

  int res = memcmp(ctx.res, accel_res, nrows_lhs*nrows_rhs*sizeof(int32_t));

  if(res == 0) {
    cout << "Test succeeded (" << testName << ")" << endl;
    //runner->printPerfSummary();
    //runner->printPerfDetails();
  } else {
    cout << "Test failed (" << testName << ")" << endl;
    cout << "Expected: " << endl;
    gemmbitserial::printmatrix(ctx.res, nrows_rhs, nrows_lhs);
    cout << "Produced: " << endl;
    gemmbitserial::printmatrix(accel_res, nrows_rhs, nrows_lhs);
    /*cout << "LHS bit serial matrix:" << endl;
    ctx.lhs.printHex();
    cout << "=======================" << endl;
    cout << "RHS bit serial matrix:" << endl;
    ctx.rhs.printHex();*/
  }

  delete [] lhs;
  delete [] rhs;
  delete [] accel_res;
  gemmbitserial::deallocGEMMContext(ctx);

  return res == 0;
}

bool test_conv(
  string testName, bismo_inference::ConvLayerDescriptor & cnv
) {
  // calculate some sizes
  int depth = cnv.ksize * cnv.ksize * cnv.ifm;
  int odim = (((cnv.idim + 2*cnv.pad) - cnv.ksize) / cnv.stride) + 1;
  // allocate input image and weight
  uint8_t * w = new uint8_t[cnv.ofm * depth];
  uint8_t * a = new uint8_t[cnv.ifm * cnv.idim * cnv.idim];
  uint8_t * a_lowered = new uint8_t[odim * odim * depth];
  // random initialization
  gemmbitserial::generateRandomVector(cnv.wbits, cnv.ofm*depth, w, cnv.wsigned);
  gemmbitserial::generateRandomVector(cnv.ibits, cnv.ifm*cnv.idim*cnv.idim, a, cnv.isigned);
  // allocate conv context
  gemmbitserial::ConvBitSerialContext ctx = gemmbitserial::allocConvBitSerialContext(
    cnv.ifm, cnv.ofm, cnv.idim, cnv.ksize, cnv.stride, cnv.pad, cnv.ibits,
    cnv.wbits, cnv.isigned, cnv.wsigned
  );
  ctx.importWeights(w);
  ctx.importActivations(a);
  gemmbitserial::gemmBitSerial(ctx.gemmctx);

  int32_t * res_hw = new int32_t[odim * odim * cnv.ofm];
  bismo_inference::init();
  bismo_inference::LayerHandle handle = bismo_inference::initConvLayer(cnv, w);
  bismo_inference::execConvLayer(handle, a, res_hw);
  bismo_inference::deinit();

  int res = memcmp(res_hw, ctx.gemmctx.res, sizeof(int32_t)*odim*odim*cnv.ofm);
  cout << "Conv test " << testName << " ";
  if(res == 0) {
    cout << "OK";
  } else {
    cout << "NOT OK";
  }
  cout << endl;
  // cleanup
  gemmbitserial::deallocConvBitSerialContext(ctx);
  delete [] res_hw;
  delete [] a_lowered;
  delete [] w;
  delete [] a;
  return res == 0;
}

bool test_small_conv(bismo_inference::HardwareConfig hwcfg) {
  bool all_OK = true;
  bismo_inference::ConvLayerDescriptor cnv;
  cnv.wbits = 1;
  cnv.ibits = 4;
  cnv.wsigned = false;
  cnv.isigned = false;
  cnv.pad = 0;
  cnv.ksize = 3;
  cnv.stride = 1;
  cnv.idim = 13;
  cnv.ifm = 64;
  cnv.ofm = 2;
  cnv.useCPULowering = true;
  all_OK = test_conv("SmallConv", cnv);

  return all_OK;
}

bool test_big_conv(bismo_inference::HardwareConfig hwcfg) {
  bool all_OK = true;
  bismo_inference::ConvLayerDescriptor cnv;
  vector<size_t> bits {2, 3};
  vector<size_t> pad {0, 1};
  vector<size_t> ksize {2, 3};
  vector<size_t> stride {1, 2};
  vector<size_t> dim {10, 13, 16};
  vector<size_t> fm {10, 16};
  size_t testnum = 0;
  cnv.wsigned = true;
  cnv.isigned = false;
  cnv.useCPULowering = true;
  for(auto & wbit: bits) {
    for(auto & ibit: bits) {
      for(auto & p: pad) {
        for(auto & k: ksize) {
          for(auto & s: stride) {
            for(auto & d: dim) {
              for(auto & ifm: fm) {
                for(auto & ofm: fm) {
                  cnv.wbits = wbit;
                  cnv.ibits = ibit;
                  cnv.pad = p;
                  cnv.ksize = k;
                  cnv.stride = s;
                  cnv.idim = d;
                  cnv.ifm = ifm;
                  cnv.ofm = ofm;
                  all_OK &= test_conv("BigConv-"+to_string(testnum), cnv);
                  testnum++;
                }
              }
            }
          }
        }
      }
    }
  }
  return all_OK;
}

bool test_binary_onchip_onetile(bismo_inference::HardwareConfig hwcfg) {
  bool all_OK = true;
  vector<size_t> k_tiles {1};
  const size_t memsize = min(hwcfg.lhsEntriesPerMem, hwcfg.rhsEntriesPerMem);

  for(auto & k_tile : k_tiles) {
    all_OK &= test(
      "binary_onchip_onetile_ktile" + to_string(k_tile),
      hwcfg.dpaDimLHS, hwcfg.dpaDimRHS, hwcfg.dpaDimCommon * k_tile
    );
  }

  return all_OK;
}

bool test_multibit_onchip_onetile(bismo_inference::HardwareConfig hwcfg) {
  bool all_OK = true;
  vector<size_t> bits {2, 4};
  vector<int> tf {0, 1};
  const size_t memsize = min(hwcfg.lhsEntriesPerMem, hwcfg.rhsEntriesPerMem);
  for(auto & lbits: bits) {
    for(auto & rbits: bits) {
      for(auto & wsgn : tf) {
        for(auto & asgn : tf) {
          const size_t maxbits = max(lbits, rbits);
          all_OK &= test(
            "multibit_onchip_onetile_" + to_string(lbits) + "bx" + to_string(rbits) + "b",
            hwcfg.dpaDimLHS, hwcfg.dpaDimRHS,
            (hwcfg.dpaDimCommon * memsize) / (maxbits * 8),
            lbits, rbits, wsgn == 1, asgn == 1
          );
        }
      }
    }
  }

  return all_OK;
}

bool test_multibit_multitile(bismo_inference::HardwareConfig hwcfg) {
  bool all_OK = true;
  all_OK &= test("2*Dm x 1024 x 2*Dn 1b x 1b", 2*hwcfg.dpaDimLHS, 2*hwcfg.dpaDimLHS, 1024, 1, 1);
  all_OK &= test("2*Dm x 1024 x 2*Dn 2b x 2b", 2*hwcfg.dpaDimLHS, 2*hwcfg.dpaDimLHS, 1024, 2, 2);
  all_OK &= test("3 x 1024 x 3 2b x 2b", 3, 3, 1024, 2, 2);
  all_OK &= test("64 x 1024 x 64 1b x 1b", 64, 64, 1024, 1, 1);
  all_OK &= test("64 x 1024 x 65 1b x 1b", 64, 65, 1024, 1, 1);
  all_OK &= test("64 x 1024 x 77 1b x 1b", 64, 77, 1024, 1, 1);
  all_OK &= test("64 x 1024 x 67 2b x 2b", 64, 67, 1024, 1, 2);
  all_OK &= test("64 x 1024 x 77 2b x 2b", 64, 77, 1024, 2, 2);
  all_OK &= test("64 x 1024 x 64 2b x 3b", 64, 64, 1024, 2, 3);
  return all_OK;
}

bool test_binary_size_independent(bismo_inference::HardwareConfig hwcfg) {
  bool all_OK = true;
  all_OK &= test(
    "binary_size_independent_", 17, 7, 11
  );

  return all_OK;
}

bool test_binary_onchip_multitile(bismo_inference::HardwareConfig hwcfg) {
  bool all_OK = true;
  vector<size_t> stripes {2, /*3,*/ 4};
  for(auto & lhs_stripes : stripes) {
    for(auto & rhs_stripes : stripes) {
      size_t ncols = hwcfg.dpaDimCommon*hwcfg.lhsEntriesPerMem / (lhs_stripes*rhs_stripes);
      size_t nrows_lhs = lhs_stripes*hwcfg.dpaDimLHS;
      size_t nrows_rhs = rhs_stripes*hwcfg.dpaDimRHS;
      all_OK &= test(
        "binary_onchip_multitile_" +
        to_string(nrows_lhs) + "x" + to_string(ncols) + "x"+ to_string(nrows_rhs),
        nrows_lhs, nrows_rhs, ncols
      );
    }
  }
  return all_OK;
}

bool test_binary_offchip_multitile(bismo_inference::HardwareConfig hwcfg) {
  bool all_OK = true;
  vector<size_t> stripes {2, /*3,*/ 4};
  for(auto & lhs_stripes : stripes) {
    for(auto & rhs_stripes : stripes) {
      size_t ncols = hwcfg.dpaDimCommon*hwcfg.lhsEntriesPerMem;
      size_t nrows_lhs = lhs_stripes*hwcfg.dpaDimLHS;
      size_t nrows_rhs = rhs_stripes*hwcfg.dpaDimRHS;
      all_OK &= test(
        "binary_offchip_multitile_" +
        to_string(nrows_lhs) + "x" + to_string(ncols) + "x"+ to_string(nrows_rhs),
        nrows_lhs, nrows_rhs, ncols
      );
    }
  }
  return all_OK;
}

bool test_binary_offchip_widerows_multitile(bismo_inference::HardwareConfig hwcfg) {
  bool all_OK = true;
  vector<size_t> lr_stripes {1, 2, 4};
  vector<size_t> z_stripes {2, 4};
  for(auto & lhs_stripe : lr_stripes) {
    for(auto & rhs_stripe : lr_stripes) {
      for(auto & z_stripe : z_stripes) {
        size_t ncols = hwcfg.dpaDimCommon*hwcfg.lhsEntriesPerMem*z_stripe;
        size_t nrows_lhs = lhs_stripe*hwcfg.dpaDimLHS;
        size_t nrows_rhs = rhs_stripe*hwcfg.dpaDimRHS;
        all_OK &= test(
          "test_binary_offchip_widerows_multitile_" +
          to_string(nrows_lhs) + "x" + to_string(ncols) + "x"+ to_string(nrows_rhs),
          nrows_lhs, nrows_rhs, ncols
        );
      }
    }
  }
  return all_OK;
}
