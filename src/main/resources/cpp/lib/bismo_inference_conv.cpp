#include "bismo_inference_internal.hpp"

namespace bismo_inference {
// parameter shape: weights[ofm][ifm][ksize][ksize]
LayerHandle initConvLayer(ConvLayerDescriptor & dsc, const uint8_t * weights, bool cpu_only) {
  gemmbitserial::ConvBitSerialContext ctx = gemmbitserial::allocConvBitSerialContext(
    dsc.ifm, dsc.ofm, dsc.idim, dsc.ksize, dsc.stride, dsc.pad, dsc.ibits,
    dsc.wbits, dsc.isigned, dsc.wsigned
  );
  BISMORT_DEBUG("[initConvLayer] Workload lhs/rhs details:");
#ifdef DEBUG
  ctx.printSummary();
#endif
  // NOTE: convbitserial uses lhs=activations rhs=weights. BISMO has the
  // rhs designated for activation buffers (e.g. the swg capability is there,
  // and the drivers are written s.t. rhs buf is shared for all layers).
  // so we swap lhs<->rhs internally in this function.
  gemmbitserial::BitSerialMatrix lhs = ctx.gemmctx.rhs;
  gemmbitserial::BitSerialMatrix rhs = ctx.gemmctx.lhs;
  // bit-serial activation buffer prior to lowering
  gemmbitserial::BitSerialMatrix abuf = ctx.abuf;
  // import the weights
  ctx.importWeights(weights);
  // create entry in layer registry
  InternalLayerDescriptor idsc;
  idsc.layerType = layerConv;
  idsc.cpu_only = cpu_only;
  idsc.cnv_ctx = ctx;
  idsc.ctx = ctx.gemmctx;
  idsc.nbytes_buf_in = 0;
  idsc.nbytes_buf_out = 0;
  idsc.cnv_dsc = dsc;
  idsc.wbase = 0;
  idsc.abase = 0;
  idsc.accel_buf_in = 0;
  idsc.accel_buf_out = 0;
  if(cpu_only) {
    // no further prep needed for cpu-only bit serial convs
    // TODO support fewifm mode for CPU only as well
    idsc.cnv_fewifm_mode = false;
  } else {
    // allocate corresponding matmul layer
    // decide whether to use few-IFM mode (first im2row, then bitserial)
    if(dsc.ifm < 32) {
      const size_t lowered_rows = ctx.out_dim * ctx.out_dim;
      const size_t lowered_cols = ctx.ifm * ctx.k * ctx.k;
      idsc.cnv_fewifm_mode = true;
      idsc.cnv_fewifm_buf = new uint8_t[lowered_cols * lowered_rows];
      MatMulLayerDescriptor mm_dsc;
      mm_dsc.wbits = dsc.wbits;
      mm_dsc.ibits = dsc.ibits;
      mm_dsc.wsigned = dsc.wsigned;
      mm_dsc.isigned = dsc.isigned;
      mm_dsc.M = ctx.ofm;
      mm_dsc.K = lowered_cols;
      mm_dsc.N = lowered_rows;
      // importWeights uses uint64_t here, so Dk must be 64 unless fixed
      assert(cfg.dpaDimCommon == 64);
      assert(dsc.useCPULowering);
      // register the matrix multiplication layer
      LayerHandle cnv_matmul_handle = initMatMulLayer(mm_dsc, weights);
      idsc.cnv_matmul_handle = cnv_matmul_handle;
      // temporary buffer to help with transposition
      idsc.transpose_result_host_buffer = new AccumType[ctx.ofm * lowered_rows];
    } else {
      idsc.cnv_fewifm_mode = false;
      MatMulLayerDescriptor mm_dsc;
      mm_dsc.wbits = dsc.wbits;
      mm_dsc.ibits = dsc.ibits;
      mm_dsc.wsigned = dsc.wsigned;
      mm_dsc.isigned = dsc.isigned;
      mm_dsc.M = lhs.nrows;
      mm_dsc.K = lhs.ncols;
      mm_dsc.N = rhs.nrows;
      // importWeights uses uint64_t here, so Dk must be 64 unless fixed
      assert(cfg.dpaDimCommon == 64);
      assert(dsc.useCPULowering);
      // register the matrix multiplication layer, skipping weight import
      LayerHandle cnv_matmul_handle = initMatMulLayer(mm_dsc, (const uint8_t *) 0);
      // set accel LHS contents from imported weights
      configMatMulLayer_Internal_SetLHS(cnv_matmul_handle, lhs);
      idsc.cnv_matmul_handle = cnv_matmul_handle;
      // temporary buffer to help with transposition
      idsc.transpose_result_host_buffer = new AccumType[lhs.nrows * rhs.nrows];
    }
    BISMORT_DEBUG("[initConvLayer] using fewifm mode = " << idsc.cnv_fewifm_mode);
  }
  LayerHandle ret = registry.size();
  registry.push_back(idsc);
  // instruction generation for the rest of the execution is done dynamically
  // in the execLayer calls for now
  return ret;
}

void execConvLayer(LayerHandle id, const uint8_t * in, int32_t * out) {
  BISMORT_DEBUG("[execConvLayer] id " << id);
  InternalLayerDescriptor dsc = registry[id];
  gemmbitserial::ConvBitSerialContext ctx = dsc.cnv_ctx;

  if(dsc.cnv_fewifm_mode) {
    // directly lower 8-byte data
    TIMER_SAMPLE();
    gemmbitserial::im2row(
      in, ctx.ifm, ctx.in_dim, ctx.in_dim, ctx.k, ctx.stride, ctx.pad,
      dsc.cnv_fewifm_buf
    );
    TIMER_SAMPLE();
    TIMER_REPORT("cpu_im2row_8b")
    // call corresponding matrix multiply
    LayerHandle cnv_matmul_handle = dsc.cnv_matmul_handle;
    InternalLayerDescriptor dsc_matmul = registry[cnv_matmul_handle];
    AccumType * targetHostBuf = dsc.transpose_result_host_buffer;
    execMatMulLayer(cnv_matmul_handle, dsc.cnv_fewifm_buf, targetHostBuf);
    // host-to-host transpose
    TIMER_SAMPLE();
    for(size_t i = 0; i < dsc_matmul.mm_dsc.N ; i++) {
      for(size_t j = 0; j < dsc_matmul.mm_dsc.M; j++) {
        out[j * dsc_matmul.mm_dsc.N + i] = targetHostBuf[i * dsc_matmul.mm_dsc.M + j];
      }
    }
    TIMER_SAMPLE();
    TIMER_REPORT("cpu_transpose");
  } else {
    // NOTE: lhs and rhs are swapped, see note in initConvLayer
    gemmbitserial::BitSerialMatrix lhs = dsc.ctx.rhs;
    gemmbitserial::BitSerialMatrix rhs = dsc.ctx.lhs;
    // using CPU lowering for now
    // TODO switch to hardware SWU
    // TODO fixes hardware lowering: everything concerning rhs here
    assert(dsc.cnv_dsc.useCPULowering);
    TIMER_SAMPLE();
    ctx.importActivations(in);
    TIMER_SAMPLE();
    TIMER_REPORT("cpu_im2row_bs");
    if(dsc.cpu_only) {
      gemmbitserial::gemmBitSerial(ctx.gemmctx);
      memcpy(out, ctx.gemmctx.res, sizeof(AccumType) * lhs.nrows * rhs.nrows);
    } else {
      LayerHandle cnv_matmul_handle = dsc.cnv_matmul_handle;
      InternalLayerDescriptor dsc_matmul = registry[cnv_matmul_handle];
      // TODO import into matmul ctx directly instead of copying
      dsc_matmul.ctx.rhs.copyFrom(rhs);
      // TODO remove all support for non-p2s conv
      platform->copyBufferHostToAccel(
        dsc_matmul.ctx.rhs.data, (void *) dsc_matmul.accel_buf_in, dsc_matmul.nbytes_buf_in
      );
      const size_t lowered_bs_act_bytes = dsc_matmul.ctx.rhs.nbits * dsc_matmul.ctx.rhs.wordsPerBitplane() * sizeof(PackedBitGroupType);
      //AccumType * targetHostBuf = out;
      AccumType * targetHostBuf = dsc.transpose_result_host_buffer;
      // call the underlying matmul implementation
      execMatMulLayer_Internal_RHSBitSerial(cnv_matmul_handle, targetHostBuf);
      // host-to-host transpose
      TIMER_SAMPLE();
      for(size_t i = 0; i < rhs.nrows; i++) {
        for(size_t j = 0; j < lhs.nrows; j++) {
          out[j * rhs.nrows + i] = targetHostBuf[i * lhs.nrows + j];
        }
      }
      TIMER_SAMPLE();
      TIMER_REPORT("cpu_transpose");

  #ifdef BISMORT_CONV_VERIFY_AGAINST_CPU
      uint64_t checksum_lhs = 0;
      uint64_t checksum_rhs = 0;
      uint32_t checksum_res = 0;
      for(size_t i = 0; i < lhs.nbits * lhs.wordsPerBitplane(); i++) checksum_lhs += lhs.data[i];
      for(size_t i = 0; i < rhs.nbits * rhs.wordsPerBitplane(); i++) checksum_rhs += rhs.data[i];
      for(size_t i = 0; i < lhs.nrows * rhs.nrows; i++) checksum_res += out[i];
      BISMORT_DEBUG("[execConvLayer] LHS checksum = " << hex << checksum_lhs << dec << endl);
      BISMORT_DEBUG("[execConvLayer] RHS checksum = " << hex << checksum_rhs << dec << endl);
      BISMORT_DEBUG("[execConvLayer] res checksum = " << hex << checksum_res << dec << endl);
  #endif

      #ifdef BISMORT_CONV_VERIFY_AGAINST_CPU
      // compute result with CPU and compare
      size_t actual_res_bytes = sizeof(AccumType) * lhs.nrows * rhs.nrows;
      gemmbitserial::gemmBitSerial(ctx.gemmctx);
      int ret = memcmp(ctx.gemmctx.res, out, actual_res_bytes);
      BISMORT_DEBUG("[execConvLayer] memcmp against golden = " << ret);
      const size_t bs_w_bytes = ctx.gemmctx.rhs.nbits * ctx.gemmctx.rhs.wordsPerBitplane() * sizeof(PackedBitGroupType);
      int cmp_wmat = memcmp(ctx.gemmctx.rhs.data, dsc_matmul.ctx.lhs.data, bs_w_bytes);
      int cmp_amat = memcmp(ctx.gemmctx.lhs.data, dsc_matmul.ctx.rhs.data, lowered_bs_act_bytes);
      BISMORT_DEBUG("[execConvLayer] W matrix memcmp = " << cmp_wmat);
      BISMORT_DEBUG("[execConvLayer] A matrix memcmp = " << cmp_amat);
      if(ret != 0) {
        cout << "expected vs found" << endl;
        for(int i = 0; i < lhs.nrows * rhs.nrows; i++) {
          if(dsc.ctx.res[i] != out[i]) {
            cout << "pos " << i << ": " << ctx.gemmctx.res[i] << " " << out[i] << endl;
          }
        }
      }
      //memcpy(out, ctx.gemmctx.res, actual_res_bytes);
      #endif
    }
  }
}
}
