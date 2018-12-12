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

#include "BISMOTests.hpp"
#include "mnist.hpp"

// read matrix dimensions from stdin and run a bit-serial matmul benchmark
void benchmark_interactive(
  WrapperRegDriver * platform, BitSerialMatMulAccelDriver * acc
) {
  while(1) {
    int rows, depth, cols, lhsbits, rhsbits, lhssigned, rhssigned;
    float secs;
    cout << "Enter rows depth cols, 0 to skip, -1 to quit " << endl;
    cin >> rows;
    if(rows == 0) {
      break;
    } else if (rows == -1) {
      exit(0);
    }
    cin >> depth >> cols;
    cout << "Enter lhs and rhs bits: " << endl;
    cin >> lhsbits >> rhsbits;
    cout << "Enter signedness (1 or 0) for lhs and rhs: " << endl;
    cin >> lhssigned >> rhssigned;

    test(
      to_string(rows) + "x" + to_string(depth) + "x" + to_string(cols) + ":" +
      to_string(lhsbits) + "b/" + to_string(rhsbits) + "b",
      platform, acc, rows, cols, depth, lhsbits, rhsbits, lhssigned, rhssigned
    );
  }
}

int main(int argc, char const *argv[]) {
  try {
    WrapperRegDriver * platform = initPlatform();
    BitSerialMatMulAccelDriver * acc = new BitSerialMatMulAccelDriver(platform);
    acc->print_hwcfg_summary();

    // Uncomment to enable interactive benchmarking:
    // benchmark_interactive(platform, acc);

  bool all_OK = true;
    all_OK &= test_binary_onchip_onetile(platform, acc);
    // all_OK &= test_binary_onchip_multitile(platform, acc);
    // all_OK &= test_binary_offchip_multitile(platform, acc);
    // all_OK &= test_binary_offchip_widerows_multitile(platform, acc);
    // all_OK &= test_qnn_mnist_0(platform, acc);

    if(all_OK) {
      cout << "All tests passed succesfully" << endl;
    } else {
      cout << "Some tests failed!" << endl;
    }

    delete acc;
    deinitPlatform(platform);
  } catch (const char * e) {
    cout << "Exception: " << e << endl;
  }
  return 0;
}
