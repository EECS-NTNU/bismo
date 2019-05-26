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

#ifndef BISMO_INFERENCE_HPP
#define BISMO_INFERENCE_HPP
#include <stdint.h>
#include <map>
#include <string>

namespace bismo_inference {
// handle representing a particular layer instance that BISMO knows
// how to execute
typedef unsigned int LayerHandle;

// struct with details of currently instantiated hardware config
// copied from BitSerialMatMulAccelDriver in order not to have that as a
// dependency
typedef struct {
  uint64_t accWidth;
  uint64_t cmdQueueEntries;
  uint64_t dpaDimCommon;
  uint64_t dpaDimLHS;
  uint64_t dpaDimRHS;
  uint64_t lhsEntriesPerMem;
  uint64_t maxShiftSteps;
  uint64_t readChanWidth;
  uint64_t rhsEntriesPerMem;
  uint64_t writeChanWidth;
  // TODO indicate coherency?
} HardwareConfig;

typedef std::map<std::string,float> InstrumentationData;

// global init/deinit for the runtime library
void init();
void deinit();

// benchmark host<->accel transfer times
void benchmark_host_accel_transfer();
// retrieve a map of all instrumentation data from the previous run
InstrumentationData getInstrumentationData();
// run a small self-test for the p2s accelerator
bool selftest_p2s();

// retrieve hardware configuration for the instance
HardwareConfig getHardwareConfig();

// handles for matrices and matrix multiply operations
typedef unsigned int MatrixHandle;
typedef unsigned int MatMulHandle;

// functions for matrix creation
MatrixHandle matrix_create(
  size_t rows, size_t cols, size_t bits, bool is_signed, bool is_transposed,
  bool is_int32
);
void * matrix_get_hostbuf(MatrixHandle mat);
void matrix_accel2host(MatrixHandle mat);
void matrix_host2accel(MatrixHandle mat);
void matrix_destroy(MatrixHandle mat);

// functions for matrix multiplication
MatMulHandle matmul_create(
  MatrixHandle lhs, MatrixHandle rhs, MatrixHandle res, bool cpu_only
);
void matmul_exec(MatMulHandle mm);
void matmul_destroy(MatMulHandle mm);
}
#endif
