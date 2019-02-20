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

namespace bismo_inference {
typedef enum {layerMatMul, layerConv, layerThres} InternalLayerType;
typedef int32_t AccumType;
typedef struct {
  InternalLayerType layerType;
  uint32_t accel_buf_in;
  uint32_t accel_buf_out;
  uint32_t accel_lhs_ptr;
  // actual workload
  gemmbitserial::GEMMContext ctx;
  // hardware workload (can be smaller when using rhs partition)
  gemmbitserial::GEMMContext hw_ctx;
  gemmbitserial::ConvBitSerialContext cnv_ctx;
  size_t nbytes_buf_in;
  size_t nbytes_buf_out;
  MatMulLayerDescriptor mm_dsc;
  ConvLayerDescriptor cnv_dsc;
  ThresLayerDescriptor thr_dsc;
  size_t wbase;
  size_t wbytes;
  size_t abase;
  std::vector<BISMOInstruction> instructions_queue;
  size_t n_act_partitions;
  AccumType * padded_result_host_buffer;
  AccumType * transpose_result_host_buffer;
  LayerHandle cnv_matmul_handle;
} InternalLayerDescriptor;

// internal global variables
// global handle for the platform and BISMO driver
extern WrapperRegDriver * platform;
extern BitSerialMatMulAccelDriver * acc;
extern HardwareCfg cfg;
extern uint32_t weightOCMBase, weightOCMBytesLeft;
extern uint32_t activationOCMBase, activationOCMBytesLeft;
extern uint32_t thresholdOCMBase, thresholdOCMBytesLeft;
extern std::vector<InternalLayerDescriptor> registry;

// internal helper functions
uint32_t allocWeightOCM(size_t nbytes);
uint32_t allocThresOCM(size_t nbytes);
size_t getNumPartitionsForActivationOCM(size_t nbytes);
void genFetchInstrs(
  std::vector<BISMOInstruction> & ins,
  size_t bram_base,
  bool lhsNotRhs,
  uint32_t dram_base,
  size_t tiles_per_row,
  size_t nbytes
);

void genMatMulInstrs_LHSPreloaded_RHSFitsOnChip(
  std::vector<BISMOInstruction> & ins,
  size_t lhs_tiles, size_t rhs_tiles, size_t k_tiles,
  size_t wbits, size_t abits,
  bool wsigned, bool asigned,
  size_t wbase_ocm, size_t abase_ocm,
  uint32_t accel_buf_rhs,
  uint32_t accel_buf_res
);

void p2s(
  const uint8_t * host_buf,
  uint32_t accel_buf,
  gemmbitserial::BitSerialMatrix & mat
);

void configMatMulLayer_Internal_SetLHS(LayerHandle id, gemmbitserial::BitSerialMatrix mat);
void execMatMulLayer_Internal_RHSBitSerial(LayerHandle id, int32_t * out);
}
#endif /* end of include guard: BISMORT_INFERENCE_HPP */
