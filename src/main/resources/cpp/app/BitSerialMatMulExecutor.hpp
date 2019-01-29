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

#include <cassert>
#include <vector>
#include <iomanip>
#include <iostream>
#include "BitSerialMatMulAccelDriver.hpp"
#include "gemmbitserial/gemmbitserial.hpp"

#define min(x,y) (x < y ? x : y)
#define max(x,y) (x > y ? x : y)
#define INVALID_CACHE_ENTRY         (uint64_t) -1

// TODO:
// - define own context allocator for the accelerator, including
// alignment requirements for lhs/rhs.
// - do not use entire GEMMContext, only need shape
// - (idea) add lockstep execution mode for debug purposes?

class BitSerialMatMulExecutor {
public:
  BitSerialMatMulExecutor(
    gemmbitserial::GEMMContext & shape,
    BitSerialMatMulAccelDriver * acc,
    WrapperRegDriver * platform
  ) {
    m_shape = shape;
    m_acc = acc;
    m_hwcfg = m_acc->hwcfg();
    m_platform = platform;
    m_bytes_to_fetch = 0;
    m_bytes_to_write = 0;
    // TODO verify alignment etc for instantiated hardware dimensions
    // allocate accelerator memory for given shape
    m_accelLHS = m_platform->allocAccelBuffer(lhsBytes());
    m_accelRHS = m_platform->allocAccelBuffer(rhsBytes());
    m_accelRes = m_platform->allocAccelBuffer(resBytes());
    // prepare the accelerator for operation
    m_acc->reset();
    m_acc->init_resource_pools();
    m_acc->useDescriptors();
    // build schedule for each stage based on shape
    build_schedule_trivial();
    // uncomment to see the generated instructions
    //printFetchQueue();
    //printExecQueue();
  }

  ~BitSerialMatMulExecutor() {
    // deinitialize allocated memory
    m_platform->deallocAccelBuffer(m_accelLHS);
    m_platform->deallocAccelBuffer(m_accelRHS);
    m_platform->deallocAccelBuffer(m_accelRes);
  }

  void setLHS(gemmbitserial::BitSerialMatrix from) {
    assert(m_shape.lhs.nrows_a == from.nrows_a);
    assert(m_shape.lhs.nbits == from.nbits);
    // copy host -> accel
    m_platform->copyBufferHostToAccel(from.data, m_accelLHS, lhsBytes());
  }

  void setRHS(gemmbitserial::BitSerialMatrix from) {
    assert(m_shape.rhs.nrows_a == from.nrows_a);
    assert(m_shape.rhs.nbits == from.nbits);
    // copy host -> accel
    m_platform->copyBufferHostToAccel(from.data, m_accelRHS, rhsBytes());
  }

  void getRes(ResultType * to) {
    // result alignment
    size_t alignedResElems = m_shape.rhs.nrows_a * m_shape.lhs.nrows_a;
    ResultType * host_res = new ResultType[alignedResElems];
    // copy aligned result into host buffer
    m_platform->copyBufferAccelToHost(m_accelRes, host_res, resBytes());
    // copy all real data (non-alignment) parts of result
    const size_t bpr = m_shape.lhs.nrows * sizeof(ResultType);
    for(size_t i = 0; i < m_shape.rhs.nrows; i++) {
      memcpy(
        &to[i * m_shape.lhs.nrows], &host_res[i * m_shape.lhs.nrows_a], bpr
      );
    }
    delete [] host_res;
  }

  void run() {
    m_acc->set_stage_enables(0, 0, 0);
    // start the cycle counter
    m_acc->perf_set_cc_enable(true);
    // enable all stages
    m_acc->set_stage_enables(1, 1, 1);
    // wait until all the results are written
    while(!allFinished()) {
      //m_acc->updateStateBreakdown();
      //m_acc->printStateBreakdown();
      //m_acc->printTokenCounts();
    };
    // disable all stages
    m_acc->set_stage_enables(0, 0, 0);
    // stop the cycle counter
    m_acc->perf_set_cc_enable(false);
    m_cycles = m_acc->perf_get_cc();
    m_acc->updateStateBreakdown();
  }

  size_t lhsBytes() const {
    return m_shape.lhs.wordsPerBitplane() * m_shape.lhs.nbits * sizeof(PackedBitGroupType);
  }

  size_t rhsBytes() const {
    return m_shape.rhs.wordsPerBitplane() * m_shape.rhs.nbits * sizeof(PackedBitGroupType);
  }

  size_t resBytes() const {
    return m_shape.lhs.nrows_a * m_shape.rhs.nrows_a * sizeof(ResultType);
  }

  // performance counters and related performance reporting functions
  // ===========================================================================
  float getNanosecondsPerCycle() const {
    return 1000.0 / m_acc->fclk_MHz();
  }

  float getLastRuntimeCycles() const {
    return (float) m_cycles;
  }

  float getLastRuntimeNanoseconds() const {
    return getLastRuntimeCycles() * getNanosecondsPerCycle();
  }

  float getWorkloadOpCount(bool inclPadding = true) const {
    if(inclPadding) {
      return 2 * m_shape.lhs.nrows_a * m_shape.rhs.nrows_a * m_shape.lhs.ncols_a;
    } else {
      return 2 * m_shape.lhs.nrows * m_shape.rhs.nrows * m_shape.lhs.ncols;
    }
  }

  float getWorkloadBinaryOpCount(bool inclPadding = true) const {
    return getWorkloadOpCount(inclPadding) * m_shape.lhs.nbits * m_shape.rhs.nbits;
  }

  float getLastRunBinaryGOPS(bool inclPadding = true) const {
    // giga-ops per second = ops per nanosecond
    return getWorkloadBinaryOpCount(inclPadding) / getLastRuntimeNanoseconds();
  }

  float getHWPeakBinaryOpsPerCycle() const {
    return 2 * m_hwcfg.dpaDimLHS * m_hwcfg.dpaDimRHS * m_hwcfg.dpaDimCommon;
  }

  float getHWPeakBinaryGOPS() const {
    return getHWPeakBinaryOpsPerCycle() / getNanosecondsPerCycle();
  }

  size_t getHWBufSizeLHS() const {
    return m_hwcfg.dpaDimLHS * m_hwcfg.lhsEntriesPerMem * m_hwcfg.dpaDimCommon/8;
  }

  size_t getHWBufSizeRHS() const {
    return m_hwcfg.dpaDimRHS * m_hwcfg.rhsEntriesPerMem * m_hwcfg.dpaDimCommon/8;
  }

  size_t getHWBufSize() const {
    return getHWBufSizeLHS() + getHWBufSizeRHS();
  }

  float getWorkloadReadOI() const {
    return getWorkloadBinaryOpCount(true) / (lhsBytes() + rhsBytes());
  }

  float getWorkloadWriteOI() const {
    return getWorkloadBinaryOpCount(true) / (resBytes());
  }

  float getWorkloadOI() const {
    return getWorkloadBinaryOpCount(true) / (lhsBytes() + rhsBytes() + resBytes());
  }

  // read bandwidth in bytes per cycle
  float getHWReadBW() const {
    return m_hwcfg.readChanWidth/8;
  }

  // write bandwidth in bytes per cycle
  float getHWWriteBW() const {
    return m_hwcfg.writeChanWidth/8;
  }

  float getHWCompBoundReadOI() const {
    return getHWPeakBinaryOpsPerCycle() / getHWReadBW();
  }

  float getHWCompBoundWriteOI() const {
    return getHWPeakBinaryOpsPerCycle() / getHWWriteBW();
  }

  void printPerfSummary() {
    std::cout << "Performance Summary ====================================" << std::endl;
    std::cout << "Total workload: " << getWorkloadBinaryOpCount(true) << " binary ops" << std::endl;
    std::cout << "Actual workload: " << getWorkloadBinaryOpCount(false) << " binary ops ";
    std::cout << "(" << 100*getWorkloadBinaryOpCount(false)/getWorkloadBinaryOpCount(true) << "%)" << std::endl;
    std::cout << "Input matrix bytes: LHS " << lhsBytes() << " RHS " << rhsBytes() << std::endl;
    std::cout << "Result matrix bytes: " << resBytes() << std::endl;
    std::cout << "HW input matrix buffer bytes: " << getHWBufSize() << std::endl;
    std::cout << "HW peak perf: " << getHWPeakBinaryGOPS() << " binary GOPS" << std::endl;
    std::cout << "HW fclk: " << m_acc->fclk_MHz() << " MHz" << std::endl;
    std::cout << "Runtime: " << getLastRuntimeCycles() << " cycles, ";
    std::cout << getLastRuntimeNanoseconds() << " ns" << std::endl;
    std::cout << "Achieved: " << getLastRunBinaryGOPS() << " binary GOPS (";
    std::cout << 100*getLastRunBinaryGOPS() / getHWPeakBinaryGOPS() << "%)" << std::endl;
    std::cout << "Workload OI read: " << getWorkloadReadOI();
    std::cout << " write: " << getWorkloadWriteOI() << std::endl;
    std::cout << "HW comp-bound OI read: " << getHWCompBoundReadOI();
    std::cout << " write: " << getHWCompBoundWriteOI() << std::endl;
    std::cout << "========================================================" << std::endl;
  }

  void printPerfDetails() {
    int colwidth = 11;
    m_acc->printStateBreakdown();

    std::cout << "Memory System ==========================================" << std::endl;
    size_t rd_total = (m_bytes_to_fetch);
    float rd_bw = (float) rd_total / getLastRuntimeCycles();
    float rd_fetchact_bw = (float) m_bytes_to_fetch / m_acc->getStateBreakdown(stgFetch, csRun);
    std::cout << "Total DRAM reads: " << rd_total << " bytes" << std::endl;
    std::cout << "Matrix DRAM reads: " << m_bytes_to_fetch << " bytes (";
    std::cout << 100*((float)(m_bytes_to_fetch)/rd_total) << "% of total)" << std::endl;
    std::cout << "HW theoretical peak read bandwidth: " << getHWReadBW() << " bytes/cycle" << std::endl;
    std::cout << "Average DRAM read bandwidth: " << rd_bw << " bytes/cycle (";
    std::cout << 100*rd_bw/getHWReadBW() << "%)" << std::endl;
    std::cout << "Fetch efficiency: " << rd_fetchact_bw << " bytes/cycle (";
    std::cout << 100*rd_fetchact_bw/getHWReadBW() << "%)" << std::endl;

    std::cout << "DRAM writes: " << m_bytes_to_write << " bytes" << std::endl;
    float wr_bw = (float)m_bytes_to_write / getLastRuntimeCycles();
    float wr_resact_bw = (float) m_bytes_to_write / m_acc->getStateBreakdown(stgResult, csRun);
    std::cout << "HW theoretical peak wr bandwidth: " << getHWWriteBW() << " bytes/cycle" << std::endl;
    std::cout << "Effective wr bandwidth: " << wr_bw << " bytes/cycle (";
    std::cout << 100*wr_bw/getHWWriteBW() << "%)" << std::endl;
    std::cout << "Result wr bandwidth: " << wr_resact_bw << " bytes/cycle (";
    std::cout << 100*wr_resact_bw/getHWWriteBW() << "%)" << std::endl;

    float exec_eff = getWorkloadBinaryOpCount(true) / ((m_acc->getStateBreakdown(stgExec, csRun) * getHWPeakBinaryOpsPerCycle()));
    std::cout << "Execute stage efficiency: " << 100*exec_eff << "%" << std::endl;
    std::cout << "========================================================" << std::endl;
  }

protected:
  uint32_t m_cycles;
  uint32_t m_bytes_to_fetch, m_bytes_to_write;

  gemmbitserial::GEMMContext m_shape;
  BitSerialMatMulAccelDriver * m_acc;
  WrapperRegDriver * m_platform;
  HardwareCfg m_hwcfg;

  void * m_accelLHS;
  void * m_accelRHS;
  void * m_accelRes;

  // whether all instruction execution has finished:
  // done when the final res instruction is completed
  bool allFinished() {
    return (m_acc->res_opcount() == 0);
  }

  void build_schedule_trivial() {
    HardwareCfg cfg = m_acc->hwcfg();
    const uint32_t dpa_y = cfg.dpaDimLHS; // DPA Y dimension
    const uint32_t dpa_x = cfg.dpaDimRHS; // DPA X dimension
    const uint32_t dpa_z = cfg.dpaDimCommon; // DPA z dimension (64)
    gemmbitserial::BitSerialMatrix lhs = m_shape.lhs; // Matrix for lhs and rhs
    gemmbitserial::BitSerialMatrix rhs = m_shape.rhs;

    SingleMMDescriptor desc;
    desc.tiles_m = lhs.nrows_a / dpa_y;
    desc.tiles_k = lhs.ncols_a / dpa_z;
    desc.tiles_n = rhs.nrows_a / dpa_x;
    desc.bits_l = lhs.nbits;
    desc.bits_r = rhs.nbits;
    desc.signed_l = lhs.issigned;
    desc.signed_r = rhs.issigned;
    desc.base_l = 0;
    desc.base_r = 0;
    desc.base_res = 0;
    desc.nbufs_res = 1;
    // need the (uint32_t)(uint64_t) to avoid -fpermissive
    // alloc'd m_ bufs are void * and gcc complains about casting down to 32-bit
    desc.dram_lhs = (uint32_t)(uint64_t) m_accelLHS;
    desc.dram_rhs = (uint32_t)(uint64_t) m_accelRHS;
    desc.dram_res = (uint32_t)(uint64_t) m_accelRes;
    m_acc->pushSingleMMDescriptor(desc);

    // update DRAM access counts
    m_bytes_to_fetch = (desc.bits_l * desc.tiles_m * dpa_y * dpa_z) / 8;
    m_bytes_to_fetch += (desc.bits_r * desc.tiles_n * dpa_x * dpa_z) / 8;
    m_bytes_to_fetch *= desc.tiles_k;
    m_bytes_to_write = m_hwcfg.dpaDimLHS * m_hwcfg.dpaDimRHS * sizeof(ResultType);
  }
};
