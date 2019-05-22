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
  idsc.nbytes_buf_in_lhs = 0;
  idsc.nbytes_buf_in_rhs = 0;
  idsc.nbytes_buf_out = 0;
  idsc.cnv_dsc = dsc;
  idsc.accel_buf_in_lhs = 0;
  idsc.accel_buf_in_rhs = 0;
  idsc.accel_buf_out = 0;
  if(cpu_only) {
    // no further prep needed for cpu-only bit serial convs
  } else {
    // allocate corresponding matmul layer
    const size_t lowered_rows = ctx.out_dim * ctx.out_dim;
    const size_t lowered_cols = ctx.ifm * ctx.k * ctx.k;
    idsc.cnv_lowering_buf = new uint8_t[lowered_cols * lowered_rows];
    MatMulLayerDescriptor mm_dsc;
    mm_dsc.wbits = dsc.wbits;
    mm_dsc.ibits = dsc.ibits;
    mm_dsc.wsigned = dsc.wsigned;
    mm_dsc.isigned = dsc.isigned;
    mm_dsc.M = ctx.ofm;
    mm_dsc.K = lowered_cols;
    mm_dsc.N = lowered_rows;
    assert(dsc.useCPULowering);
    // register the matrix multiplication layer
    LayerHandle cnv_matmul_handle = initMatMulLayer(mm_dsc, weights);
    idsc.cnv_matmul_handle = cnv_matmul_handle;
    // temporary buffer to help with transposition
    idsc.transpose_result_host_buffer = new AccumType[ctx.ofm * lowered_rows];
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

  if(dsc.cpu_only) {
    // NOTE: lhs and rhs are swapped, see note in initConvLayer
    gemmbitserial::BitSerialMatrix lhs = dsc.ctx.rhs;
    gemmbitserial::BitSerialMatrix rhs = dsc.ctx.lhs;
    TIMER_SAMPLE();
    ctx.importActivations(in);
    TIMER_SAMPLE();
    TIMER_REPORT("cpu_im2row_bs");
    TIMER_SAMPLE();
    gemmbitserial::gemmBitSerial(ctx.gemmctx);
    TIMER_SAMPLE();
    TIMER_REPORT("cpu_gemmbs");
    memcpy(out, ctx.gemmctx.res, sizeof(AccumType) * lhs.nrows * rhs.nrows);
    #ifdef BISMORT_CONV_VERIFY_AGAINST_CPU
    verifyConv(ctx, in, out);
    #endif
  } else {
    // directly lower 8-byte data
    TIMER_SAMPLE();
    gemmbitserial::im2row(
      in, ctx.ifm, ctx.in_dim, ctx.in_dim, ctx.k, ctx.stride, ctx.pad,
      dsc.cnv_lowering_buf
    );
    TIMER_SAMPLE();
    TIMER_REPORT("cpu_im2row_8b")
    // call corresponding matrix multiply
    LayerHandle cnv_matmul_handle = dsc.cnv_matmul_handle;
    InternalLayerDescriptor dsc_matmul = registry[cnv_matmul_handle];
    AccumType * targetHostBuf = dsc.transpose_result_host_buffer;
    execMatMulLayer(cnv_matmul_handle, dsc.cnv_lowering_buf, targetHostBuf);
    // host-to-host transpose
    TIMER_SAMPLE();
    for(size_t i = 0; i < dsc_matmul.mm_dsc.N ; i++) {
      for(size_t j = 0; j < dsc_matmul.mm_dsc.M; j++) {
        out[j * dsc_matmul.mm_dsc.N + i] = targetHostBuf[i * dsc_matmul.mm_dsc.M + j];
      }
    }
    TIMER_SAMPLE();
    TIMER_REPORT("cpu_transpose");
  }
}
}
