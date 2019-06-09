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

#include "bismo_rt_internal.hpp"

namespace bismo_rt {

// hardware metrics (non-workload-dependent)
float getHWPeakBinaryOpsPerCycle() {
  return 2 * cfg.dpaDimLHS * cfg.dpaDimRHS * cfg.dpaDimCommon;
}

float getHWPeakBinaryGOPS() {
  return getHWPeakBinaryOpsPerCycle() / getNanosecondsPerCycle();
}

size_t getHWBufSizeLHS() {
  return cfg.dpaDimLHS * cfg.lhsEntriesPerMem * cfg.dpaDimCommon/8;
}

size_t getHWBufSizeRHS() {
  return cfg.dpaDimRHS * cfg.rhsEntriesPerMem * cfg.dpaDimCommon/8;
}

size_t getHWBufSize() {
  return getHWBufSizeLHS() + getHWBufSizeRHS();
}

float getNanosecondsPerCycle() {
  return 1000.0 / acc->fclk_MHz();
}

float getLastRuntimeCycles() {
  return (float) acc->perf_get_cc();
}

float getLastRuntimeNanoseconds() {
  return getLastRuntimeCycles() * getNanosecondsPerCycle();
}

// read bandwidth in bytes per cycle
float getHWReadBW() {
  return cfg.readChanWidth/8;
}

// write bandwidth in bytes per cycle
float getHWWriteBW() {
  return cfg.writeChanWidth/8;
}

float getHWCompBoundReadOI() {
  return getHWPeakBinaryOpsPerCycle() / getHWReadBW();
}

float getHWCompBoundWriteOI() {
  return getHWPeakBinaryOpsPerCycle() / getHWWriteBW();
}

}
