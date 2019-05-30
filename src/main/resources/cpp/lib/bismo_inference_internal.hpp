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
// internal global variables
// global handle for the platform and BISMO driver
extern WrapperRegDriver * platform;
extern BitSerialMatMulAccelDriver * acc;
extern HardwareCfg cfg;
extern uint32_t accel_p2s_bitpar_buffer;
extern uint8_t * host_p2s_bitpar_buffer;
//extern std::vector<InternalLayerDescriptor> registry;
extern InstrumentationData instrumentationData;
#ifdef BISMORT_INSTRUMENTATION
extern std::chrono::time_point<std::chrono::high_resolution_clock> time_prev, time_now;
#endif
// internal helper functions
void p2s(
  const uint8_t * host_buf_src,   // input matrix buffer (source)
  uint32_t accel_buf_dst,         // output matrix buffer (destination)
  size_t nrows, size_t ncols,     // matrix size
  size_t nbits,                   // actual bits per element in source matrix
  bool issigned,                  // whether source matrix is signed
  bool zeropad = false,           // use zero instead of random padding
  size_t row_align = 1            // align rows to multiple of this
);
}
#endif /* end of include guard: BISMORT_INFERENCE_HPP */
