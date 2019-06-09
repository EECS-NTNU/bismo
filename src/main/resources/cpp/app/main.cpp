// Copyright (c) 2018 Norwegian University of Science and Technology (NTNU)
// Copyright (c) 2019 Xilinx
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
// * Neither the name of BISMO nor the names of its
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

#include "BISMOTests.hpp"
#include "benchmark.hpp"

int main(int argc, char const *argv[]) {
  try {
    if(argc != 2) {
      cout << "Run with cmdline argument: " << endl;
      cout << "t to run tests" << endl;
      cout << "i to run interactive benchmarking" << endl;
      cout << "b to run batch-mode benchmarking" << endl;
      return -1;
    }
    if(argv[1][0] == 'i') {
      benchmark_gemm_interactive();
    } else if(argv[1][0] == 'b') {
      benchmark_gemm_batch();
    } else if(argv[1][0] == 't') {
      bool all_OK = true;
      bismo_rt::init();
      bismo_rt::HardwareConfig hwcfg = bismo_rt::getHardwareConfig();
      all_OK &= bismo_rt::selftest_shared_buffer();
      all_OK &= bismo_rt::selftest_matrix();
      all_OK &= bismo_rt::selftest_p2s();
      bismo_rt::deinit();
      // following tests call init/deinit themselves
      all_OK &= test_binary_onchip_onetile(hwcfg);
      all_OK &= test_multibit_onchip_onetile(hwcfg);
      all_OK &= test_multibit_multitile(hwcfg);
      if(all_OK) {
        cout << "All tests passed succesfully" << endl;
      } else {
        cout << "Some tests failed!" << endl;
      }
    }
  } catch (const char * e) {
    cout << "Exception: " << e << endl;
  }
  return 0;
}
