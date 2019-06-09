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

#ifndef BISMORT_HPP
#define BISMORT_HPP
#include <stdint.h>
#include <map>
#include <string>

namespace bismo_rt {
// global init/deinit for the runtime library
void init();
void deinit();

// descriptor for the shape/size/precision of a matrix multiplication
typedef struct {
  uint8_t wbits;  // bits per weight
  uint8_t ibits;  // bits per input
  bool wsigned;   // whether weights are signed
  bool isigned;   // whether inputs are signed
  uint32_t M;     // rows of left-hand-side (weight) matrix
  uint32_t K;     // common dimension (columns)
  uint32_t N;     // rows of right-hand-side (weight) matrix
} MatMulDescriptor;
// handle representing a particular layer instance that BISMO knows
// how to execute
typedef uint64_t LayerHandle;
// initialize matrix multiplication and return handle
LayerHandle initMatMul(MatMulDescriptor & dsc);
// get host-accessible buffers associated with layer
uint8_t * getLayerLHSBuffer(LayerHandle id);
uint8_t * getLayerRHSBuffer(LayerHandle id);
int32_t * getLayerResBuffer(LayerHandle id);
// synchronize buffers associated with layer:
// ensure that input (LHS/RHS) buffers are up-to-date on the accelerator
void syncLayerLHSBuffer(LayerHandle id);
void syncLayerRHSBuffer(LayerHandle id);
// ensure that result buffer is up-to-date on the host
void syncLayerResBuffer(LayerHandle id);
// execute layer with given handle
void execMatMul(LayerHandle id);
// struct representing all instrumentation data from the previous run
typedef std::map<std::string,float> InstrumentationData;
// retrieve a map of all instrumentation data from the previous run
InstrumentationData getInstrumentationData(LayerHandle id);
// destroy layer with given handle
void deinitMatMul(LayerHandle id);

// struct with details of currently instantiated hardware config
// copied from BitSerialMatMulAccelDriver in order not to have that as a
// dependency
typedef struct {
  uint64_t accWidth;          // number of bits in accumulator, 32 for now
  uint64_t cmdQueueEntries;   // size of the command queue for instructions
  uint64_t dpaDimCommon;      // Dk
  uint64_t dpaDimLHS;         // Dm
  uint64_t dpaDimRHS;         // Dn
  uint64_t lhsEntriesPerMem;  // number of entries in LHS on-chip memory
  uint64_t maxShiftSteps;     // obsolete, do not use
  uint64_t readChanWidth;     // max bits read from DRAM per cycle
  uint64_t rhsEntriesPerMem;  // number of entries in RHS on-chip memory
  uint64_t writeChanWidth;    // max bits written to DRAM per cycle
} HardwareConfig;
// retrieve hardware configuration for the instance
HardwareConfig getHardwareConfig();
// benchmark host<->accel transfer times
void benchmark_host_accel_transfer();
// run a small self-test for the p2s accelerator
bool selftest_p2s();
// run self-test for buffer copy operations
bool selftest_shared_buffer();
// run self-test for matrix pad and copy operations
bool selftest_matrix();
}
#endif
