#include "bismo_inference_internal.hpp"

namespace bismo_inference {

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

// workload-specific functions as members of InternalLayerDescriptor
#ifdef BISMORT_INSTRUMENTATION
size_t InternalLayerDescriptor::lhsBytes() const {
  return ctx.lhs.wordsPerBitplane() * ctx.lhs.nbits * sizeof(PackedBitGroupType);
}

size_t InternalLayerDescriptor::rhsBytes() const {
  return ctx.rhs.wordsPerBitplane() * ctx.rhs.nbits * sizeof(PackedBitGroupType);
}

size_t InternalLayerDescriptor::resBytes() const {
  return ctx.lhs.nrows_a * ctx.rhs.nrows_a * sizeof(ResultType);
}

size_t InternalLayerDescriptor::getNumBytesToFetch() const {
  return rhsBytes();
}

size_t InternalLayerDescriptor::getNumBytesToWrite() const {
  return resBytes();
}

float InternalLayerDescriptor::getWorkloadOpCount(bool inclPadding) const {
  if(inclPadding) {
    return 2 * ctx.lhs.nrows_a * ctx.rhs.nrows_a * ctx.lhs.ncols_a;
  } else {
    return 2 * ctx.lhs.nrows * ctx.rhs.nrows * ctx.lhs.ncols;
  }
}

float InternalLayerDescriptor::getWorkloadBinaryOpCount(bool inclPadding) const {
  return getWorkloadOpCount(inclPadding) * ctx.lhs.nbits * ctx.rhs.nbits;
}

float InternalLayerDescriptor::getLastRunBinaryGOPS(bool inclPadding) const {
  // giga-ops per second = ops per nanosecond
  return getWorkloadBinaryOpCount(inclPadding) / getLastRuntimeNanoseconds();
}

float InternalLayerDescriptor::getWorkloadReadOI() const {
  return getWorkloadBinaryOpCount(true) / (lhsBytes() + rhsBytes());
}

float InternalLayerDescriptor::getWorkloadWriteOI() const {
  return getWorkloadBinaryOpCount(true) / (resBytes());
}

float InternalLayerDescriptor::getWorkloadOI() const {
  return getWorkloadBinaryOpCount(true) / (lhsBytes() + rhsBytes() + resBytes());
}

// read bandwidth in bytes per cycle
float InternalLayerDescriptor::getHWReadBW() const {
  return cfg.readChanWidth/8;
}

// write bandwidth in bytes per cycle
float InternalLayerDescriptor::getHWWriteBW() const {
  return cfg.writeChanWidth/8;
}

float InternalLayerDescriptor::getHWCompBoundReadOI() const {
  return getHWPeakBinaryOpsPerCycle() / getHWReadBW();
}

float InternalLayerDescriptor::getHWCompBoundWriteOI() const {
  return getHWPeakBinaryOpsPerCycle() / getHWWriteBW();
}

void InternalLayerDescriptor::printPerfSummary() {
  instrumentationData["workload_total_binops"] = getWorkloadBinaryOpCount(true);
  instrumentationData["workload_actual_binops"] = getWorkloadBinaryOpCount(false);
  instrumentationData["lhs_bytes"] = lhsBytes();
  instrumentationData["rhs_bytes"] = rhsBytes();
  instrumentationData["res_bytes"] = resBytes();
  instrumentationData["buf_size_bytes"] = getHWBufSize();
  instrumentationData["peak_perf_binops"] = getHWPeakBinaryGOPS();
  instrumentationData["fclk_mhz"] = acc->fclk_MHz();
  instrumentationData["hw_cycles"] = getLastRuntimeCycles();
  instrumentationData["hw_achieved_binops"] = getLastRunBinaryGOPS();
  instrumentationData["workload_read_oi"] = getWorkloadReadOI();
  instrumentationData["workload_write_oi"] = getWorkloadWriteOI();
  instrumentationData["hw_peak_read_oi"] = getHWCompBoundReadOI();
  instrumentationData["hw_peak_write_oi"] = getHWCompBoundWriteOI();
#ifdef BISMORT_INSTRUMENTATION_VERBOSE
  std::cout << "Performance Summary ====================================" << std::endl;
  std::cout << "Total workload: " << instrumentationData["workload_total_binops"] << " binary ops" << std::endl;
  std::cout << "Actual workload: " << instrumentationData["workload_actual_binops"] << " binary ops ";
  std::cout << "(" << 100*instrumentationData["workload_total_binops"]/instrumentationData["workload_actual_binops"] << "%)" << std::endl;
  std::cout << "Input matrix bytes: LHS " << instrumentationData["lhs_bytes"] << " RHS " << instrumentationData["rhs_bytes"] << std::endl;
  std::cout << "Result matrix bytes: " << instrumentationData["res_bytes"] << std::endl;
  std::cout << "HW input matrix buffer bytes: " << instrumentationData["buf_size_bytes"] << std::endl;
  std::cout << "HW peak perf: " << instrumentationData["peak_perf_binops"] << " binary GOPS" << std::endl;
  std::cout << "HW fclk: " << instrumentationData["fclk_mhz"] << " MHz" << std::endl;
  std::cout << "Runtime: " << instrumentationData["hw_cycles"] << " cycles, ";
  std::cout << getLastRuntimeNanoseconds() << " ns" << std::endl;
  std::cout << "Achieved: " << instrumentationData["hw_achieved_binops"] << " binary GOPS (";
  std::cout << 100*instrumentationData["hw_achieved_binops"] / instrumentationData["peak_perf_binops"] << "%)" << std::endl;
  std::cout << "Workload OI read: " << instrumentationData["workload_read_oi"];
  std::cout << " write: " << instrumentationData["workload_write_oi"] << std::endl;
  std::cout << "HW comp-bound OI read: " << instrumentationData["hw_peak_read_oi"];
  std::cout << " write: " << instrumentationData["hw_peak_write_oi"] << std::endl;
  std::cout << "========================================================" << std::endl;
#endif
}

void InternalLayerDescriptor::printPerfDetails() {
  size_t rd_total = (getNumBytesToFetch());
  float rd_bw = (float) rd_total / getLastRuntimeCycles();
  float rd_fetchact_bw = (float) getNumBytesToFetch() / acc->getStateBreakdown(stgFetch, csRun);
  float wr_bw = (float)getNumBytesToWrite() / getLastRuntimeCycles();
  float wr_resact_bw = (float) getNumBytesToWrite() / acc->getStateBreakdown(stgResult, csRun);
  float exec_eff = getWorkloadBinaryOpCount(true) / ((acc->getStateBreakdown(stgExec, csRun) * getHWPeakBinaryOpsPerCycle()));
  instrumentationData["dram_read_bytes"] = rd_total;
  instrumentationData["hw_peak_read_bw"] = getHWReadBW();
  instrumentationData["dram_write_bytes"] = getNumBytesToWrite();
  instrumentationData["hw_peak_write_bw"] = getHWWriteBW();
  instrumentationData["fetch_idle"] = acc->getStateBreakdown(stgFetch, csGetCmd);
  instrumentationData["exec_idle"] = acc->getStateBreakdown(stgExec, csGetCmd);
  instrumentationData["result_idle"] = acc->getStateBreakdown(stgResult, csGetCmd);
  instrumentationData["fetch_run"] = acc->getStateBreakdown(stgFetch, csRun);
  instrumentationData["exec_run"] = acc->getStateBreakdown(stgExec, csRun);
  instrumentationData["result_run"] = acc->getStateBreakdown(stgResult, csRun);
  instrumentationData["fetch_snd"] = acc->getStateBreakdown(stgFetch, csSend);
  instrumentationData["exec_snd"] = acc->getStateBreakdown(stgExec, csSend);
  instrumentationData["result_snd"] = acc->getStateBreakdown(stgResult, csSend);
  instrumentationData["fetch_rcv"] = acc->getStateBreakdown(stgFetch, csReceive);
  instrumentationData["exec_rcv"] = acc->getStateBreakdown(stgExec, csReceive);
  instrumentationData["result_rcv"] = acc->getStateBreakdown(stgResult, csReceive);
#ifdef BISMORT_INSTRUMENTATION_VERBOSE
  int colwidth = 11;
  acc->printStateBreakdown();
  std::cout << "Memory System ==========================================" << std::endl;
  std::cout << "Total DRAM reads: " << instrumentationData["dram_read_bytes"] << " bytes" << std::endl;
  std::cout << "HW theoretical peak read bandwidth: " << instrumentationData["hw_peak_read_bw"] << " bytes/cycle" << std::endl;
  std::cout << "Average DRAM read bandwidth: " << rd_bw << " bytes/cycle (";
  std::cout << 100*rd_bw/getHWReadBW() << "%)" << std::endl;
  std::cout << "Fetch efficiency: " << rd_fetchact_bw << " bytes/cycle (";
  std::cout << 100*rd_fetchact_bw/getHWReadBW() << "%)" << std::endl;
  std::cout << "DRAM writes: " << instrumentationData["dram_write_bytes"] << " bytes" << std::endl;
  std::cout << "HW theoretical peak wr bandwidth: " << instrumentationData["hw_peak_write_bw"] << " bytes/cycle" << std::endl;
  std::cout << "Effective wr bandwidth: " << wr_bw << " bytes/cycle (";
  std::cout << 100*wr_bw/getHWWriteBW() << "%)" << std::endl;
  std::cout << "Result wr bandwidth: " << wr_resact_bw << " bytes/cycle (";
  std::cout << 100*wr_resact_bw/getHWWriteBW() << "%)" << std::endl;
  std::cout << "Execute stage efficiency: " << 100*exec_eff << "%" << std::endl;
  std::cout << "========================================================" << std::endl;
#endif
}
#endif
}
