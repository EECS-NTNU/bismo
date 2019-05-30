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
