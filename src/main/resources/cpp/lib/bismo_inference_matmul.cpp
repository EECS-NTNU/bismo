/*
#include "bismo_inference_internal.hpp"

namespace bismo_inference {
// initialize layer of given type and return handle
// parameter shape: weights[M][K]
LayerHandle initMatMulLayer(MatMulLayerDescriptor & dsc, const uint8_t * weights, bool cpu_only) {
  // allocate OCM space for weights
  gemmbitserial::GEMMContext ctx = acc->allocGEMMContext(
    dsc.M, dsc.K, dsc.N, dsc.wbits, dsc.ibits, dsc.wsigned, dsc.isigned
  );
  BISMORT_DEBUG("[initMatMulLayer] Workload lhs/rhs details:");
#ifdef DEBUG
  ctx.lhs.printSummary();
  ctx.rhs.printSummary();
#endif
  // convert weights to bit serial
  // don't really care about performance here since this is one-off
  memset(ctx.lhs.data, 0, ctx.lhs.nbits*ctx.lhs.wordsPerBitplane() * sizeof(PackedBitGroupType));
  memset(ctx.rhs.data, 0, ctx.rhs.nbits*ctx.rhs.wordsPerBitplane() * sizeof(PackedBitGroupType));
  // allow skipping weight import (if e.g. Internal_SetLHS will be used)
  if(weights != 0) {
    ctx.lhs.importRegular(weights);
  }
  // create entry in layer registry and return layer handle
  LayerHandle ret = registry.size();
  InternalLayerDescriptor idsc;
  idsc.cpu_only = cpu_only;
  idsc.layerType = layerMatMul;
  idsc.ctx = ctx;
  idsc.mm_dsc = dsc;
  if(cpu_only) {
    // set unused fields to 0 for cpu only bsmatmul
    idsc.nbytes_buf_in_rhs = 0;
    idsc.nbytes_buf_in_lhs = 0;
    idsc.nbytes_buf_out = 0;
    idsc.padded_result_host_buffer = 0;
    idsc.accel_buf_in_lhs = 0;
    idsc.accel_buf_in_rhs = 0;
    idsc.accel_buf_out = 0;
  } else {
    // must have room for at least one stripe per bit position, as this is the
    // granularity we at which we do RHS tiling
    bool rhs_tile_fits_in_ocm = (acc->get_rhs_total_BRAM_bytes()) >= FETCHEXEC_TOKENS*(ctx.rhs.nbits * ctx.rhs.wordsPerRow() * cfg.dpaDimRHS * sizeof(PackedBitGroupType));
    bool rhs_tile_is_one_fetchblock = (ctx.rhs.nbits * ctx.rhs.wordsPerRow() * cfg.dpaDimRHS * sizeof(PackedBitGroupType)) <= FETCH_BLOCK_MAX;
    if(!rhs_tile_is_one_fetchblock || !rhs_tile_fits_in_ocm) {
      throw "RHS tile is too large and not currently supported in runtime library.";
    }
    bool lhs_tile_fits_in_ocm = (acc->get_lhs_total_BRAM_bytes()) >= FETCHEXEC_TOKENS*(ctx.lhs.nbits * ctx.lhs.wordsPerRow() * cfg.dpaDimLHS * sizeof(PackedBitGroupType));
    bool lhs_tile_is_one_fetchblock = (ctx.lhs.nbits * ctx.lhs.wordsPerRow() * cfg.dpaDimLHS * sizeof(PackedBitGroupType)) <= FETCH_BLOCK_MAX;
    if(!lhs_tile_is_one_fetchblock || !lhs_tile_fits_in_ocm) {
      throw "LHS tile is too large and not currently supported in runtime library.";
    }
    size_t wbytes = ctx.lhs.wordsPerBitplane() * ctx.lhs.nbits * sizeof(PackedBitGroupType);
    size_t abytes = ctx.rhs.wordsPerBitplane() * ctx.rhs.nbits * sizeof(PackedBitGroupType);
    size_t resbytes = ctx.lhs.nrows_a * ctx.rhs.nrows_a * sizeof(AccumType);
    const size_t lhs_tiles = ctx.lhs.nrows_a /  cfg.dpaDimLHS;
    const size_t rhs_tiles = ctx.rhs.nrows_a /  cfg.dpaDimRHS;
    const size_t k_tiles = ctx.rhs.ncols_a /  cfg.dpaDimCommon;
    const size_t wbits = ctx.lhs.nbits;
    const size_t abits = ctx.rhs.nbits;
    const bool wsigned = ctx.lhs.issigned;
    const bool asigned = ctx.rhs.issigned;
    // fill out rest of descriptor
    idsc.nbytes_buf_in_rhs = abytes;
    idsc.nbytes_buf_in_lhs = wbytes;
    idsc.nbytes_buf_out = ctx.lhs.nrows_a * ctx.rhs.nrows_a * sizeof(AccumType) ;
    idsc.padded_result_host_buffer = new AccumType[ctx.lhs.nrows_a * ctx.rhs.nrows_a];
    idsc.accel_buf_in_lhs = (uint32_t)(uint64_t)platform->allocAccelBuffer(wbytes);
    idsc.accel_buf_in_rhs = (uint32_t)(uint64_t)platform->allocAccelBuffer(abytes);
    idsc.accel_buf_out = (uint32_t)(uint64_t)platform->allocAccelBuffer(resbytes);
    BISMORT_DEBUG("[initMatMulLayer] accel_buf_in_lhs: " << (idsc.accel_buf_in_lhs) << " for " << wbytes << " bytes");
    BISMORT_DEBUG("[initMatMulLayer] accel_buf_in_rhs: " << (idsc.accel_buf_in_rhs) << " for " << abytes << " bytes");
    BISMORT_DEBUG("[initMatMulLayer] accel_buf_out: " << (idsc.accel_buf_out) << " for " << resbytes << " bytes");
    // fill out descriptor for matmul
    // create an instruction generation descriptor
    SingleMMDescriptor instrgen_dsc;
    instrgen_dsc.tiles_m = lhs_tiles;
    instrgen_dsc.tiles_k = k_tiles;
    instrgen_dsc.tiles_n = rhs_tiles;
    instrgen_dsc.bits_l = wbits;
    instrgen_dsc.bits_r = abits;
    instrgen_dsc.signed_l = wsigned;
    instrgen_dsc.signed_r = asigned;
    instrgen_dsc.base_l = 0;
    instrgen_dsc.base_r = 0;
    instrgen_dsc.base_res = 0;
    instrgen_dsc.nbufs_fetch_exec_log2 = FETCHEXEC_TOKENS_LOG2;
    instrgen_dsc.dram_lhs = idsc.accel_buf_in_lhs;
    instrgen_dsc.dram_rhs = idsc.accel_buf_in_rhs;
    instrgen_dsc.dram_res = idsc.accel_buf_out;
    idsc.instrgen_dsc = instrgen_dsc;
  }
  registry.push_back(idsc);

  if(!cpu_only) {
    // set the contents of the LHS matrix for the accelerator
    configMatMulLayer_Internal_SetLHS(ret, idsc.ctx.lhs);
  }
  BISMORT_DEBUG("[initMatMulLayer] registered new matmul layer with id " << ret);
  return ret;
}

// replace the LHS matrix for a layer
// specified matrix shape and initialized matrix shape must be the same
void configMatMulLayer_Internal_SetLHS(LayerHandle id, gemmbitserial::BitSerialMatrix mat) {
  TIMER_SAMPLE();
  // instrgen doesn't support only fetching LHS, use manual feed
  acc->useDirectInstructionFeed();
  InternalLayerDescriptor dsc = registry[id];
  // all code here is for hw exec only
  assert(!dsc.cpu_only);
  // allocate DRAM buffer and copy bit-serial weights there
  // TODO optimization: can use a single DRAM buffer whose size is the largest
  // weight buffer since this is done only once per layer
  uint32_t accel_buf_in_lhs = dsc.accel_buf_in_lhs;
  const size_t wbytes = dsc.nbytes_buf_in_lhs;
#ifdef DEBUG
  BISMORT_DEBUG("[configMatMulLayer_Internal_SetLHS] Source matrix:");
  mat.printSummary();
  BISMORT_DEBUG("[configMatMulLayer_Internal_SetLHS] HW shape matrix:");
  dsc.ctx.lhs.printSummary();
#endif
  if(dsc.ctx.lhs.data != mat.data) {
    dsc.ctx.lhs.copyFrom(mat);
  }
  platform->copyBufferHostToAccel((void *)dsc.ctx.lhs.data, (void *)accel_buf_in_lhs, wbytes);

  BISMORT_DEBUG("[configMatMulLayer_Internal_SetLHS] weight init done");
  TIMER_SAMPLE();
  TIMER_REPORT("cpu_setLHS");
}

// execute layer with given handle
// in and out are assumed to be preallocated to appropriate buffer sizes,
// depending on the type of layer
void execMatMulLayer(LayerHandle id, const uint8_t * in, int32_t * out) {
  BISMORT_DEBUG("[execMatMulLayer] id " << id);
  InternalLayerDescriptor dsc = registry[id];

  if(dsc.cpu_only) {
    dsc.ctx.rhs.importRegular(in);
    gemmbitserial::gemmBitSerial(dsc.ctx);
    memcpy(out, dsc.ctx.res, sizeof(int32_t)*dsc.ctx.lhs.nrows*dsc.ctx.rhs.nrows);
  } else {
    p2s(
      in, dsc.accel_buf_in_rhs, dsc.ctx.rhs.nrows, dsc.ctx.rhs.ncols,
      dsc.ctx.rhs.nbits, dsc.ctx.rhs.issigned, false, cfg.dpaDimRHS
    );
    execMatMulLayer_Internal_RHSBitSerial(id, out);
  }
#ifdef BISMORT_MATMUL_VERIFY_AGAINST_CPU
  uint64_t checksum_lhs = 0;
  uint64_t checksum_rhs = 0;
  uint32_t checksum_res = 0;
  for(size_t i = 0; i < dsc.ctx.lhs.nbits * dsc.ctx.lhs.wordsPerBitplane(); i++) checksum_lhs += dsc.ctx.lhs.data[i];
  for(size_t i = 0; i < dsc.ctx.rhs.nbits * dsc.ctx.rhs.wordsPerBitplane(); i++) checksum_rhs += dsc.ctx.rhs.data[i];
  for(size_t i = 0; i < dsc.ctx.lhs.nrows * dsc.ctx.rhs.nrows; i++) checksum_res += dsc.ctx.res[i];
  BISMORT_DEBUG("[execMatMulLayer] LHS checksum = " << hex << checksum_lhs << dec << endl);
  BISMORT_DEBUG("[execMatMulLayer] RHS checksum = " << hex << checksum_rhs << dec << endl);
  BISMORT_DEBUG("[execMatMulLayer] res checksum = " << hex << checksum_res << dec << endl);
#endif
}

// asssumes that the RHS matrix is already in bit-serial format in the ctx
void execMatMulLayer_Internal_RHSBitSerial(LayerHandle id, int32_t * out) {
  BISMORT_DEBUG("[execMatMulLayer_Internal_RHSBitSerial] id " << id);
  InternalLayerDescriptor dsc = registry[id];
  // all code here is for hw exec only
  assert(!dsc.cpu_only);
  gemmbitserial::BitSerialMatrix lhs = dsc.ctx.lhs;
  gemmbitserial::BitSerialMatrix rhs = dsc.ctx.rhs;
  AccumType * padded_result_host_buffer = dsc.padded_result_host_buffer;
  instrumentationData["workload_M"] = lhs.nrows;
  instrumentationData["workload_K"] = lhs.ncols;
  instrumentationData["workload_N"] = rhs.nrows;
  instrumentationData["workload_M_pad"] = lhs.nrows_a;
  instrumentationData["workload_K_pad"] = lhs.ncols_a;
  instrumentationData["workload_N_pad"] = rhs.nrows_a;
  instrumentationData["workload_lbits"] = lhs.nbits;
  instrumentationData["workload_rbits"] = rhs.nbits;
  uint32_t rptr = dsc.accel_buf_out;
  acc->set_stage_enables(0, 0, 0);
  acc->useDescriptors();
  // feed the instrgen descriptor
  acc->pushSingleMMDescriptor(dsc.instrgen_dsc);
  // HACK: make sure at least one op has appeared before checking for completion
  // proper way to fix this is to singal completion from accel explicitly
  while(acc->res_opcount() == 0) {};
  TIMER_SAMPLE();
#ifdef BISMORT_INSTRUMENTATION
  acc->perf_set_cc_enable(1);
#endif
  // enable all stages
  acc->set_stage_enables(1, 1, 1);
  // wait until all writes are completed
  while(acc->res_opcount() != 0) {
    //BISMORT_DEBUG("[execMatMulLayer] waiting for exec, ops f/e/r: " << acc->fetch_opcount() << " " << acc->exec_opcount() << " " << acc->res_opcount());
  };
#ifdef BISMORT_INSTRUMENTATION
  acc->perf_set_cc_enable(0);
#endif
  TIMER_SAMPLE();
  TIMER_REPORT("run_hwexec_us");
  // copy padded result buffer to host
  TIMER_SAMPLE();
  platform->copyBufferAccelToHost((void *)dsc.accel_buf_out, (void *) padded_result_host_buffer, dsc.nbytes_buf_out);
  TIMER_SAMPLE();
  TIMER_REPORT("cpu_accel2host");
#ifdef BISMORT_INSTRUMENTATION
  acc->updateStateBreakdown();
  dsc.printPerfSummary();
  dsc.printPerfDetails();
#endif
  TIMER_SAMPLE();
  // get rid of padding as needed
  if((lhs.nrows_a == lhs.nrows) && (rhs.nrows_a == rhs.nrows)) {
    // no padding was necessary, copy directly
    memcpy(out, padded_result_host_buffer, lhs.nrows_a * rhs.nrows_a * sizeof(AccumType));
  } else {
    // accelerator computed a padded result matrix, copy actual parts only
    size_t nbytes_row_nonpadded = sizeof(AccumType) * lhs.nrows;
    for(size_t row = 0; row < rhs.nrows; row++) {
      size_t ind_padded = row * lhs.nrows_a;
      size_t ind_actual = row * lhs.nrows;
      memcpy((void *)(&out[ind_actual]), &padded_result_host_buffer[ind_padded], nbytes_row_nonpadded);
    }
  }
  TIMER_SAMPLE();
  TIMER_REPORT("cpu_rmpad");

#ifdef BISMORT_MATMUL_VERIFY_AGAINST_CPU
  // compute result with CPU and compare
  size_t actual_res_bytes = sizeof(AccumType) * lhs.nrows * rhs.nrows;
  gemmbitserial::gemmBitSerial(dsc.ctx);
  int ret = memcmp(dsc.ctx.res, out, actual_res_bytes);
  BISMORT_DEBUG("[execMatMulLayer] memcmp against golden = " << ret);
  if(ret != 0) {
    cout << "expected vs found" << endl;
    for(int i = 0; i < dsc.ctx.lhs.nrows * dsc.ctx.rhs.nrows; i++) {
      if(dsc.ctx.res[i] != out[i]) {
        cout << "pos " << i << ": " << dsc.ctx.res[i] << " " << out[i] << endl;
      }
    }
  }
#endif
}

}
*/
