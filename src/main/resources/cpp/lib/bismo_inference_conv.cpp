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
  idsc.n_act_partitions = 0;
  idsc.accel_buf_in = 0;
  idsc.accel_buf_out = 0;
  if(cpu_only) {
    // no further prep needed for cpu-only bit serial convs
  } else {
    // allocate corresponding matmul layer
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
    // register the matrix multiplication layer
    LayerHandle cnv_matmul_handle = initMatMulLayer(mm_dsc, weights);
    // set accel LHS contents from imported weights
    configMatMulLayer_Internal_SetLHS(cnv_matmul_handle, lhs);
    idsc.cnv_matmul_handle = cnv_matmul_handle;
    // temporary buffer to help with transposition
    idsc.transpose_result_host_buffer = new AccumType[lhs.nrows * rhs.nrows];
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
  TIMER_REPORT("execConvLayer CPU lowering");
  if(dsc.cpu_only) {
    gemmbitserial::gemmBitSerial(ctx.gemmctx);
    memcpy(out, ctx.gemmctx.res, sizeof(AccumType) * lhs.nrows * rhs.nrows);
  } else {
    LayerHandle cnv_matmul_handle = dsc.cnv_matmul_handle;
    InternalLayerDescriptor dsc_matmul = registry[cnv_matmul_handle];
    // TODO import into matmul ctx directly instead of copying
    dsc_matmul.ctx.rhs.copyFrom(rhs);
    const size_t lowered_bs_act_bytes = dsc_matmul.ctx.rhs.nbits * dsc_matmul.ctx.rhs.wordsPerBitplane() * sizeof(PackedBitGroupType);
    TIMER_SAMPLE();
    TIMER_REPORT("execConvLayer copy to matmul rhs");
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
    TIMER_REPORT("transpose result on CPU");

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
