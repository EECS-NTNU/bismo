#ifndef BISMORT_INFERENCE_HPP
#define BISMORT_INFERENCE_HPP
#include "bismo_inference.hpp"
#include "BitSerialMatMulAccelDriver.hpp"
#include <vector>
#include <string.h>
#include <algorithm>
#include <chrono>

#include "bismo_inference_options.hpp"

#ifdef DEBUG
#define BISMORT_DEBUG(x) cout << x << endl;
#else
#define BISMORT_DEBUG(x) ;
#endif

#ifndef BISMORT_INSTRUMENTATION
#define TIMER_INIT() ;
#define TIMER_SAMPLE() ;
#define TIMER_REPORT(name) ;
#else
#define TIMER_INIT() std::chrono::time_point<std::chrono::high_resolution_clock> time_prev = std::chrono::high_resolution_clock::now(); std::chrono::time_point<std::chrono::high_resolution_clock> time_now = std::chrono::high_resolution_clock::now();
#define TIMER_SAMPLE() time_prev = time_now; time_now = std::chrono::high_resolution_clock::now();
#ifdef BISMORT_INSTRUMENTATION_VERBOSE
#define TIMER_REPORT(name) cout << "[Instrumentation] " << name << " = " << std::chrono::duration_cast<std::chrono::microseconds>(time_now-time_prev).count() << " us" << endl; instrumentationData[name] = (float) (std::chrono::duration_cast<std::chrono::microseconds>(time_now-time_prev).count());
#else
#define TIMER_REPORT(name) instrumentationData[name] = (float) (std::chrono::duration_cast<std::chrono::microseconds>(time_now-time_prev).count());
#endif
#endif

namespace bismo_inference {
typedef enum {layerMatMul, layerConv, layerThres} InternalLayerType;
typedef int32_t AccumType;
typedef struct {
  InternalLayerType layerType;
  uint32_t accel_buf_in_lhs;
  uint32_t accel_buf_in_rhs;
  uint32_t accel_buf_out;
  // actual workload
  gemmbitserial::GEMMContext ctx;
  gemmbitserial::ConvBitSerialContext cnv_ctx;
  size_t nbytes_buf_in_lhs;
  size_t nbytes_buf_in_rhs;
  size_t nbytes_buf_out;
  MatMulLayerDescriptor mm_dsc;
  ConvLayerDescriptor cnv_dsc;
  ThresLayerDescriptor thr_dsc;
  SingleMMDescriptor instrgen_dsc;
  AccumType * padded_result_host_buffer;
  AccumType * transpose_result_host_buffer;
  LayerHandle cnv_matmul_handle;
  bool cpu_only;
  uint8_t * cnv_lowering_buf;
#ifdef BISMORT_INSTRUMENTATION
  size_t getNumBytesToFetch() const;
  size_t getNumBytesToWrite() const;
  size_t lhsBytes() const;
  size_t rhsBytes() const;
  size_t resBytes() const;
  float getWorkloadOpCount(bool inclPadding = true) const;
  float getWorkloadBinaryOpCount(bool inclPadding = true) const;
  float getLastRunBinaryGOPS(bool inclPadding = true) const;
  float getActualReadOI() const;
  float getActualWriteOI() const;
  float getWorkloadReadOI() const;
  float getWorkloadWriteOI() const;
  float getWorkloadOI() const;
  float getHWReadBW() const;
  float getHWWriteBW() const;
  float getHWCompBoundReadOI() const;
  float getHWCompBoundWriteOI() const;
  void printPerfSummary();
  void printPerfDetails();
#endif
} InternalLayerDescriptor;

// internal global variables
// global handle for the platform and BISMO driver
extern WrapperRegDriver * platform;
extern BitSerialMatMulAccelDriver * acc;
extern HardwareCfg cfg;
extern uint32_t accel_p2s_bitpar_buffer;
extern uint8_t * host_p2s_bitpar_buffer;
extern std::vector<InternalLayerDescriptor> registry;
extern InstrumentationData instrumentationData;
#ifdef BISMORT_INSTRUMENTATION
extern std::chrono::time_point<std::chrono::high_resolution_clock> time_prev, time_now;
#endif
// internal helper functions
void genFetchInstrs(
  std::vector<BISMOInstruction> & ins,
  size_t bram_base,
  bool lhsNotRhs,
  uint32_t dram_base,
  size_t tiles_per_row,
  size_t nbytes
);

void p2s(
  const uint8_t * host_buf_src,   // input matrix buffer (source)
  uint32_t accel_buf_dst,         // output matrix buffer (destination)
  size_t nrows, size_t ncols,     // matrix size
  size_t nbits,                   // actual bits per element in source matrix
  bool issigned,                  // whether source matrix is signed
  bool zeropad = false,           // use zero instead of random padding
  size_t row_align = 1            // align rows to multiple of this
);

void configMatMulLayer_Internal_SetLHS(LayerHandle id, gemmbitserial::BitSerialMatrix mat);
void execMatMulLayer_Internal_RHSBitSerial(LayerHandle id, int32_t * out);

// utility functions for instrumentation
// hardware metrics (non-workload-dependent)
float getHWPeakBinaryOpsPerCycle();
float getHWPeakBinaryGOPS();
size_t getHWBufSizeLHS();
size_t getHWBufSizeRHS();
size_t getHWBufSize();
float getNanosecondsPerCycle();
float getLastRuntimeCycles();
float getLastRuntimeNanoseconds();

// functions for internal verification
void verifyConv(
  gemmbitserial::ConvBitSerialContext ctx,
  const uint8_t * in,
  const int32_t * out
);
}
#endif /* end of include guard: BISMORT_INFERENCE_HPP */
