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

#include "bismo_rt_matmul.hpp"
#include <iostream>

namespace bismo_rt {
MatrixMultiply::MatrixMultiply(
  Matrix<uint8_t> * lhs, Matrix<uint8_t> * rhs, Matrix<int32_t> * res,
  bool allow_gemmbitserial
) {
  m_lhs = lhs;
  m_rhs = rhs;
  m_res = res;
  m_allow_gemmbitserial = allow_gemmbitserial;
  // ensure sizes are compatible
  if(m_lhs->inner() != m_rhs->inner()) {
    throw "LHS/RHS dimensions are incompatible";
  }
  if((m_lhs->outer() != m_res->inner()) || (m_rhs->outer() != m_res->outer())) {
    throw "Result dimensions are incompatible with input(s)";
  }
  // alloc gemmbitserial context if desired
  if(allow_gemmbitserial) {
    m_cpu_ctx = gemmbitserial::allocGEMMContext(
      m_lhs->outer(), m_lhs->inner(), m_rhs->outer(),
      m_lhs->bits(), m_rhs->bits(), m_lhs->is_signed(), m_rhs->is_signed()
    );
  }
  const size_t tiles_m = m_lhs->outer_a() / cfg.dpaDimLHS;
  const size_t tiles_k = m_lhs->inner_a() / cfg.dpaDimCommon;
  const size_t tiles_n = m_rhs->outer_a() / cfg.dpaDimRHS;
  const size_t lhs_stripe_nbytes = m_lhs->bitserial_nbytes() / tiles_m;
  const size_t rhs_stripe_nbytes = m_rhs->bitserial_nbytes() / tiles_n;
  // must have room for at least one stripe per bit position, as this is the
  // granularity we at which we do RHS tiling
  const bool rhs_tile_fits_in_ocm = (acc->get_rhs_total_BRAM_bytes()) >= FETCHEXEC_TOKENS*rhs_stripe_nbytes;
  const bool rhs_tile_is_one_fetchblock = (rhs_stripe_nbytes <= FETCH_BLOCK_MAX);
  if(!rhs_tile_is_one_fetchblock || !rhs_tile_fits_in_ocm) {
    throw "RHS is too large and not currently supported in runtime library.";
  }
  const bool lhs_tile_fits_in_ocm = (acc->get_lhs_total_BRAM_bytes()) >= FETCHEXEC_TOKENS*lhs_stripe_nbytes;
  const bool lhs_tile_is_one_fetchblock = lhs_stripe_nbytes <= FETCH_BLOCK_MAX;
  if(!lhs_tile_is_one_fetchblock || !lhs_tile_fits_in_ocm) {
    throw "LHS is too large and not currently supported in runtime library.";
  }
  // create and fill in the descriptor
  m_igen_dsc.tiles_m = tiles_m;
  m_igen_dsc.tiles_k = tiles_k;
  m_igen_dsc.tiles_n = tiles_n;
  m_igen_dsc.bits_l = m_lhs->bits();
  m_igen_dsc.bits_r = m_rhs->bits();
  m_igen_dsc.signed_l = m_lhs->is_signed();
  m_igen_dsc.signed_r = m_rhs->is_signed();
  m_igen_dsc.base_l = 0;
  m_igen_dsc.base_r = 0;
  m_igen_dsc.base_res = 0;
  m_igen_dsc.nbufs_fetch_exec_log2 = FETCHEXEC_TOKENS_LOG2;
  m_igen_dsc.dram_lhs = m_lhs->bitserial_accelbuf();
  m_igen_dsc.dram_rhs = m_rhs->bitserial_accelbuf();
  m_igen_dsc.dram_res = m_res->accelbuf();
};

// note: deallocation of MatrixMultiply does NOT free the LHS/RHS/res matrices
MatrixMultiply::~MatrixMultiply() {
  if(m_allow_gemmbitserial) {
    gemmbitserial::deallocGEMMContext(m_cpu_ctx);
  }
};

bool MatrixMultiply::has_cpu_ctx() const {
  return m_allow_gemmbitserial;
}

gemmbitserial::GEMMContext MatrixMultiply::getCPUContext() {
  if(m_allow_gemmbitserial) {
    return m_cpu_ctx;
  } else {
    throw "MatrixMultiply not constructed with allow_gemmbitserial";
  }
}

void MatrixMultiply::exec() {
  acc->set_stage_enables(0, 0, 0);
  acc->useDescriptors();
  // feed the instrgen descriptor
  acc->pushSingleMMDescriptor(m_igen_dsc);
  // HACK: make sure at least one op has appeared before checking for completion
  // proper way to fix this is to singal completion from accel explicitly
  while(acc->res_opcount() == 0) {};
  // start the cycle counter
  acc->perf_set_cc_enable(1);
  // enable all stages
  acc->set_stage_enables(1, 1, 1);
  // wait until all writes are completed
  while(acc->res_opcount() != 0) {};
  // stop the cycle counter
  acc->perf_set_cc_enable(0);
  acc->set_stage_enables(0, 0, 0);
};

size_t MatrixMultiply::M() const {
  return m_lhs->outer();
}

size_t MatrixMultiply::K() const {
  return m_lhs->inner();
}

size_t MatrixMultiply::N() const {
  return m_rhs->outer();
}

size_t MatrixMultiply::lhsBytes() const {
  return m_lhs->bitserial_nbytes();
}

size_t MatrixMultiply::rhsBytes() const {
  return m_rhs->bitserial_nbytes();
}

size_t MatrixMultiply::resBytes() const {
  return m_res->elems_a() * m_res->elem_nbytes();
}

size_t MatrixMultiply::getNumBytesToFetch() const {
  // note that the number of bytes to fetch depends on the tiling strategy
  // our current tiling strategy looks like this:
  // (see FetchInstrGen.cpp for HLS implementation)
  // foreach n in n_tiles:
  //    load slice n into on-chip memory
  //    foreach m in m_tiles:
  //      load slice m into on-chip memory
  //      process the loaded slices
  // m is slices of the LHS matrix and n is slices of the RHS matrix
  // thus, RHS gets loaded only once, but LHS is loaded multiple (n_tiles) times
  return rhsBytes() + (lhsBytes() * m_igen_dsc.tiles_n);
}

size_t MatrixMultiply::getNumBytesToWrite() const {
  return resBytes();
}

float MatrixMultiply::getWorkloadOpCount(bool inclPadding) const {
  if(inclPadding) {
    return 2 * m_lhs->elems_a() * m_rhs->outer_a();
  } else {
    return 2 * m_lhs->elems() * m_rhs->outer();
  }
}

float MatrixMultiply::getWorkloadBinaryOpCount(bool inclPadding) const {
  return getWorkloadOpCount(inclPadding) * m_lhs->bits() * m_rhs->bits();
}

float MatrixMultiply::getLastRunBinaryGOPS(bool inclPadding) const {
  // giga-ops per second = ops per nanosecond
  return getWorkloadBinaryOpCount(inclPadding) / getLastRuntimeNanoseconds();
}

float MatrixMultiply::getWorkloadReadOI() const {
  return getWorkloadBinaryOpCount(true) / (lhsBytes() + rhsBytes());
}

float MatrixMultiply::getWorkloadWriteOI() const {
  return getWorkloadBinaryOpCount(true) / (resBytes());
}

float MatrixMultiply::getActualReadOI() const {
  return getWorkloadBinaryOpCount(true) / (getNumBytesToFetch());
}

float MatrixMultiply::getActualWriteOI() const {
  return getWorkloadBinaryOpCount(true) / (getNumBytesToWrite());
}

float MatrixMultiply::getWorkloadOI() const {
  return getWorkloadBinaryOpCount(true) / (lhsBytes() + rhsBytes() + resBytes());
}

void MatrixMultiply::perfSummary() {
  instrumentationData["workload_total_binops"] = getWorkloadBinaryOpCount(true);
  instrumentationData["workload_actual_binops"] = getWorkloadBinaryOpCount(false);
  instrumentationData["workload_lhs_bytes"] = lhsBytes();
  instrumentationData["workload_rhs_bytes"] = rhsBytes();
  instrumentationData["workload_res_bytes"] = resBytes();
  instrumentationData["hw_buf_size_bytes"] = getHWBufSize();
  instrumentationData["hw_peak_perf_binops"] = getHWPeakBinaryGOPS();
  instrumentationData["hw_fclk_mhz"] = acc->fclk_MHz();
  instrumentationData["workload_read_oi"] = getWorkloadReadOI();
  instrumentationData["workload_write_oi"] = getWorkloadWriteOI();
  instrumentationData["actual_read_oi"] = getActualReadOI();
  instrumentationData["actual_write_oi"] = getActualWriteOI();
  instrumentationData["hw_peak_read_oi"] = getHWCompBoundReadOI();
  instrumentationData["hw_peak_write_oi"] = getHWCompBoundWriteOI();
  instrumentationData["run_cycles"] = getLastRuntimeCycles();
  instrumentationData["run_achieved_binops"] = getLastRunBinaryGOPS();
#ifdef BISMORT_INSTRUMENTATION_VERBOSE
  std::cout << "Performance Summary ====================================" << std::endl;
  std::cout << "Total workload: " << instrumentationData["workload_total_binops"] << " binary ops" << std::endl;
  std::cout << "Useful workload: " << instrumentationData["workload_actual_binops"] << " binary ops ";
  std::cout << "(" << 100*instrumentationData["workload_total_binops"]/instrumentationData["workload_actual_binops"] << "%)" << std::endl;
  std::cout << "Input matrix bytes: LHS " << instrumentationData["workload_lhs_bytes"] << " RHS " << instrumentationData["workload_rhs_bytes"] << std::endl;
  std::cout << "Result matrix bytes: " << instrumentationData["workload_res_bytes"] << std::endl;
  std::cout << "HW input matrix buffer bytes: " << instrumentationData["hw_buf_size_bytes"] << std::endl;
  std::cout << "HW peak perf: " << instrumentationData["hw_peak_perf_binops"] << " binary GOPS" << std::endl;
  std::cout << "HW fclk: " << instrumentationData["hw_fclk_mhz"] << " MHz" << std::endl;
  std::cout << "Workload OI read: " << instrumentationData["workload_read_oi"];
  std::cout << " write: " << instrumentationData["workload_write_oi"] << std::endl;
  std::cout << "Implementation OI read: " << instrumentationData["actual_read_oi"];
  std::cout << " write: " << instrumentationData["actual_write_oi"] << std::endl;
  std::cout << "HW comp-bound OI read: " << instrumentationData["hw_peak_read_oi"];
  std::cout << " write: " << instrumentationData["hw_peak_write_oi"] << std::endl;
  std::cout << "Achieved: " << instrumentationData["run_achieved_binops"] << " binary GOPS (";
  std::cout << 100*instrumentationData["run_achieved_binops"] / instrumentationData["hw_peak_perf_binops"] << "%)" << std::endl;
  std::cout << "Runtime: " << instrumentationData["run_cycles"] << " cycles, ";
  std::cout << getLastRuntimeNanoseconds() << " ns" << std::endl;
  std::cout << "========================================================" << std::endl;
#endif
}

void MatrixMultiply::perfDetails() {
  size_t rd_total = (getNumBytesToFetch());
  float rd_bw = (float) rd_total / getLastRuntimeCycles();
  float rd_fetchact_bw = (float) getNumBytesToFetch() / acc->getStateBreakdown(stgFetch, csRun);
  float wr_bw = (float)getNumBytesToWrite() / getLastRuntimeCycles();
  float wr_resact_bw = (float) getNumBytesToWrite() / acc->getStateBreakdown(stgResult, csRun);
  float exec_eff = getWorkloadBinaryOpCount(true) / ((acc->getStateBreakdown(stgExec, csRun) * getHWPeakBinaryOpsPerCycle()));
  instrumentationData["workload_dram_read_bytes"] = rd_total;
  instrumentationData["workload_dram_write_bytes"] = getNumBytesToWrite();
  instrumentationData["hw_peak_read_bw"] = getHWReadBW();
  instrumentationData["hw_peak_write_bw"] = getHWWriteBW();
  // per-stage state breakdown
  instrumentationData["stg_fetch_idle"] = acc->getStateBreakdown(stgFetch, csGetCmd);
  instrumentationData["stg_exec_idle"] = acc->getStateBreakdown(stgExec, csGetCmd);
  instrumentationData["stg_result_idle"] = acc->getStateBreakdown(stgResult, csGetCmd);
  instrumentationData["stg_fetch_run"] = acc->getStateBreakdown(stgFetch, csRun);
  instrumentationData["stg_exec_run"] = acc->getStateBreakdown(stgExec, csRun);
  instrumentationData["stg_result_run"] = acc->getStateBreakdown(stgResult, csRun);
  instrumentationData["stg_fetch_snd"] = acc->getStateBreakdown(stgFetch, csSend);
  instrumentationData["stg_exec_snd"] = acc->getStateBreakdown(stgExec, csSend);
  instrumentationData["stg_result_snd"] = acc->getStateBreakdown(stgResult, csSend);
  instrumentationData["stg_fetch_rcv"] = acc->getStateBreakdown(stgFetch, csReceive);
  instrumentationData["stg_exec_rcv"] = acc->getStateBreakdown(stgExec, csReceive);
  instrumentationData["stg_result_rcv"] = acc->getStateBreakdown(stgResult, csReceive);
  // derived per-stage efficiency metrics
  instrumentationData["run_eff%_fetch"] = 100*rd_fetchact_bw/getHWReadBW();
  instrumentationData["run_eff%_exec"] = 100*exec_eff;
  instrumentationData["run_eff%_result"] = 100*wr_resact_bw/getHWWriteBW();
#ifdef BISMORT_INSTRUMENTATION_VERBOSE
  int colwidth = 11;
  acc->printStateBreakdown();
  std::cout << "Memory System ==========================================" << std::endl;
  std::cout << "Total DRAM reads: " << instrumentationData["workload_dram_read_bytes"] << " bytes" << std::endl;
  std::cout << "HW theoretical peak read bandwidth: " << instrumentationData["hw_peak_read_bw"] << " bytes/cycle" << std::endl;
  std::cout << "Average DRAM read bandwidth: " << rd_bw << " bytes/cycle (";
  std::cout << 100*rd_bw/getHWReadBW() << "%)" << std::endl;
  std::cout << "Fetch efficiency: " << rd_fetchact_bw << " bytes/cycle (";
  std::cout << 100*rd_fetchact_bw/getHWReadBW() << "%)" << std::endl;
  std::cout << "DRAM writes: " << instrumentationData["workload_dram_write_bytes"] << " bytes" << std::endl;
  std::cout << "HW theoretical peak wr bandwidth: " << instrumentationData["hw_peak_write_bw"] << " bytes/cycle" << std::endl;
  std::cout << "Effective wr bandwidth: " << wr_bw << " bytes/cycle (";
  std::cout << 100*wr_bw/getHWWriteBW() << "%)" << std::endl;
  std::cout << "Result wr bandwidth: " << wr_resact_bw << " bytes/cycle (";
  std::cout << 100*wr_resact_bw/getHWWriteBW() << "%)" << std::endl;
  std::cout << "Execute stage efficiency: " << 100*exec_eff << "%" << std::endl;
  std::cout << "========================================================" << std::endl;
#endif
}
}
