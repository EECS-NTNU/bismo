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
  std::cout << "Performance Summary ====================================" << std::endl;
  std::cout << "Total workload: " << getWorkloadBinaryOpCount(true) << " binary ops" << std::endl;
  std::cout << "Actual workload: " << getWorkloadBinaryOpCount(false) << " binary ops ";
  std::cout << "(" << 100*getWorkloadBinaryOpCount(false)/getWorkloadBinaryOpCount(true) << "%)" << std::endl;
  std::cout << "Input matrix bytes: LHS " << lhsBytes() << " RHS " << rhsBytes() << std::endl;
  std::cout << "Result matrix bytes: " << resBytes() << std::endl;
  std::cout << "HW input matrix buffer bytes: " << getHWBufSize() << std::endl;
  std::cout << "HW peak perf: " << getHWPeakBinaryGOPS() << " binary GOPS" << std::endl;
  std::cout << "HW fclk: " << acc->fclk_MHz() << " MHz" << std::endl;
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

void InternalLayerDescriptor::printPerfDetails() {
  int colwidth = 11;
  acc->printStateBreakdown();

  std::cout << "Memory System ==========================================" << std::endl;
  size_t rd_total = (getNumBytesToFetch());
  float rd_bw = (float) rd_total / getLastRuntimeCycles();
  float rd_fetchact_bw = (float) getNumBytesToFetch() / acc->getStateBreakdown(stgFetch, csRun);
  std::cout << "Total DRAM reads: " << rd_total << " bytes" << std::endl;
  std::cout << "Matrix DRAM reads: " << getNumBytesToFetch() << " bytes (";
  std::cout << 100*((float)(getNumBytesToFetch())/rd_total) << "% of total)" << std::endl;
  std::cout << "HW theoretical peak read bandwidth: " << getHWReadBW() << " bytes/cycle" << std::endl;
  std::cout << "Average DRAM read bandwidth: " << rd_bw << " bytes/cycle (";
  std::cout << 100*rd_bw/getHWReadBW() << "%)" << std::endl;
  std::cout << "Fetch efficiency: " << rd_fetchact_bw << " bytes/cycle (";
  std::cout << 100*rd_fetchact_bw/getHWReadBW() << "%)" << std::endl;

  std::cout << "DRAM writes: " << getNumBytesToWrite() << " bytes" << std::endl;
  float wr_bw = (float)getNumBytesToWrite() / getLastRuntimeCycles();
  float wr_resact_bw = (float) getNumBytesToWrite() / acc->getStateBreakdown(stgResult, csRun);
  std::cout << "HW theoretical peak wr bandwidth: " << getHWWriteBW() << " bytes/cycle" << std::endl;
  std::cout << "Effective wr bandwidth: " << wr_bw << " bytes/cycle (";
  std::cout << 100*wr_bw/getHWWriteBW() << "%)" << std::endl;
  std::cout << "Result wr bandwidth: " << wr_resact_bw << " bytes/cycle (";
  std::cout << 100*wr_resact_bw/getHWWriteBW() << "%)" << std::endl;

  float exec_eff = getWorkloadBinaryOpCount(true) / ((acc->getStateBreakdown(stgExec, csRun) * getHWPeakBinaryOpsPerCycle()));
  std::cout << "Execute stage efficiency: " << 100*exec_eff << "%" << std::endl;
  std::cout << "========================================================" << std::endl;
}
#endif
}
