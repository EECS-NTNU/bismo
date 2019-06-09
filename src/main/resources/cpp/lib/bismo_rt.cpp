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
TIMER_INIT();
// global handle for the platform and BISMO driver
WrapperRegDriver * platform;
BitSerialMatMulAccelDriver * acc;
HardwareCfg cfg;
std::map<std::string,float> instrumentationData;

// global init/deinit for the runtime library
void init() {
  platform = initPlatform();
  acc = new BitSerialMatMulAccelDriver(platform);
  acc->reset();
  // currently the runtime is implemented with direct instruction feed
  // will switch to descriptors when the correct generators are impl'd
  acc->init_resource_pools();
  acc->useDirectInstructionFeed();
  cfg = acc->hwcfg();
  // allocate shared buffer for p2s
  accel_p2s_bitpar_buffer = (uint32_t)(uint64_t) platform->allocAccelBuffer(BISMORT_P2S_BITPAR_BYTES);
  host_p2s_bitpar_buffer = new uint8_t[BISMORT_P2S_BITPAR_BYTES];
}

void deinit() {
  delete acc;
  delete [] host_p2s_bitpar_buffer;
  platform->deallocAccelBuffer((void *) accel_p2s_bitpar_buffer);
  deinitPlatform(platform);
}

void benchmark_host_accel_transfer() {
  std::vector<size_t> vsize {1, 2, 4, 8, 16, 32};
  for(auto & s : vsize) {
    size_t nbytes = s * 1024;
    uint8_t * hostbuf = new uint8_t[nbytes];
    memset(hostbuf, 0x1f, nbytes);
    void * accelbuf = platform->allocAccelBuffer(nbytes);
    TIMER_SAMPLE();
    platform->copyBufferHostToAccel(hostbuf, accelbuf, nbytes);
    TIMER_SAMPLE();
    TIMER_REPORT("host2accel");
    TIMER_SAMPLE();
    platform->copyBufferAccelToHost(accelbuf, hostbuf, nbytes);
    TIMER_SAMPLE();
    TIMER_REPORT("accel2host");
    delete [] hostbuf;
    platform->deallocAccelBuffer(accelbuf);
  }
}

HardwareConfig getHardwareConfig() {
  HardwareConfig ret;
  ret.accWidth = cfg.accWidth;
  ret.cmdQueueEntries = cfg.cmdQueueEntries;
  ret.dpaDimCommon = cfg.dpaDimCommon;
  ret.dpaDimLHS = cfg.dpaDimLHS;
  ret.dpaDimRHS = cfg.dpaDimRHS;
  ret.lhsEntriesPerMem = cfg.lhsEntriesPerMem;
  ret.maxShiftSteps = cfg.maxShiftSteps;
  ret.readChanWidth = cfg.readChanWidth;
  ret.rhsEntriesPerMem = cfg.rhsEntriesPerMem;
  ret.writeChanWidth = cfg.writeChanWidth;
  return ret;
}
}
