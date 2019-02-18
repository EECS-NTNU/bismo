#include "bismo_inference_internal.hpp"

namespace bismo_inference {

// initialize layer of given type and return handle
// parameter shape: weights[M][K]
LayerHandle initMatMulLayer(MatMulLayerDescriptor & dsc, const uint8_t * weights) {
  // allocate OCM space for weights
  gemmbitserial::GEMMContext ctx = acc->allocGEMMContext(
    dsc.M, dsc.K, dsc.N, dsc.wbits, dsc.ibits, dsc.wsigned, dsc.isigned
  );
  BISMORT_DEBUG("[initMatMulLayer] Workload lhs/rhs details:");
#ifdef DEBUG
  ctx.lhs.printSummary();
  ctx.rhs.printSummary();
#endif
  size_t abytes_workload_total = ctx.rhs.wordsPerBitplane() * ctx.rhs.nbits * sizeof(PackedBitGroupType);
  size_t n_act_partitions = getNumPartitionsForActivationOCM(abytes_workload_total);
  gemmbitserial::GEMMContext hw_ctx;
  if(n_act_partitions > 1) {
    assert(0); // TODO partition sizing here
  } else {
    hw_ctx = ctx;
  }
  // TODO this needs to be checked for fetch width != exec width
  // (see exec_to_fetch_width_ratio in BitSerialMatMulExecutor)
  assert(cfg.dpaDimCommon == cfg.readChanWidth);
  size_t wbytes = hw_ctx.lhs.wordsPerBitplane() * hw_ctx.lhs.nbits * sizeof(PackedBitGroupType);
  size_t abytes = hw_ctx.rhs.wordsPerBitplane() * hw_ctx.rhs.nbits * sizeof(PackedBitGroupType);
  size_t resbytes = hw_ctx.lhs.nrows_a * hw_ctx.rhs.nrows_a * sizeof(AccumType);
  uint32_t wbase = allocWeightOCM(wbytes);
  uint32_t abase = 0; // all activations use the same OCM buffer
  // convert weights to bit serial
  // don't really care about performance here since this is one-off
  hw_ctx.lhs.importRegular(weights);
  ctx.lhs.importRegular(weights);
  // allocate DRAM buffer and copy bit-serial weights there
  // TODO optimization: can use a single DRAM buffer whose size is the largest
  // weight buffer since this is done only once per layer
  uint32_t accel_lhs_ptr = (uint32_t)(uint64_t)platform->allocAccelBuffer(wbytes);
  platform->copyBufferHostToAccel((void *)hw_ctx.lhs.data, (void *)accel_lhs_ptr, wbytes);
  // create instruction sequence to fetch weights into OCM
  std::vector<BISMOInstruction> instrFetchWeights;
  genFetchInstrs(instrFetchWeights, wbase, true, accel_lhs_ptr, hw_ctx.lhs.ncols_a / cfg.dpaDimCommon, wbytes);
  acc->set_stage_enables(0, 0, 0);
  for(auto & fi : instrFetchWeights) {
    acc->pushInstruction(fi);
  }
  BISMORT_DEBUG("[initMatMulLayer] created weight init instructions: " << acc->fetch_opcount());
  // launch weight fetch and wait until complete
  acc->set_stage_enables(1, 0, 0);
  while(acc->fetch_opcount() != 0) {
    BISMORT_DEBUG("[initMatMulLayer] waiting for weight init, ops f/e/r: " << acc->fetch_opcount() << " " << acc->exec_opcount() << " " << acc->res_opcount());
  };
  acc->set_stage_enables(0, 0, 0);
  BISMORT_DEBUG("[initMatMulLayer] weight init done");
  // create entry in layer registry and return layer handle
  InternalLayerDescriptor idsc;
  idsc.layerType = layerMatMul;
  idsc.ctx = ctx;
  idsc.hw_ctx = hw_ctx;
  idsc.nbytes_buf_in = abytes;
  idsc.nbytes_buf_out = hw_ctx.lhs.nrows_a * hw_ctx.rhs.nrows_a * sizeof(AccumType) ;
  idsc.mm_dsc = dsc;
  idsc.wbase = wbase;
  idsc.abase = abase;
  idsc.n_act_partitions = n_act_partitions;
  idsc.accel_buf_in = (uint32_t)(uint64_t)platform->allocAccelBuffer(abytes);
  BISMORT_DEBUG("[initMatMulLayer] accel_buf_in: " << (idsc.accel_buf_in) << " for " << abytes << " bytes");
  // TODO optimization: can use common buffer for all results, since 1 layer
  // at a time
  idsc.accel_buf_out = (uint32_t)(uint64_t)platform->allocAccelBuffer(resbytes);
  BISMORT_DEBUG("[initMatMulLayer] accel_buf_out: " << (idsc.accel_buf_out) << " for " << resbytes << " bytes");
  LayerHandle ret = registry.size();
  BISMORT_DEBUG("[initMatMulLayer] registered new matmul layer with id " << ret);
  const size_t lhs_tiles = hw_ctx.lhs.nrows_a /  cfg.dpaDimLHS;
  const size_t rhs_tiles = hw_ctx.rhs.nrows_a /  cfg.dpaDimRHS;
  const size_t k_tiles = hw_ctx.rhs.ncols_a /  cfg.dpaDimCommon;
  const size_t wbits = hw_ctx.lhs.nbits;
  const size_t abits = hw_ctx.rhs.nbits;
  const bool wsigned = hw_ctx.lhs.issigned;
  const bool asigned = hw_ctx.rhs.issigned;
  genMatMulInstrs_LHSPreloaded_RHSFitsOnChip(
    idsc.instructions_queue, lhs_tiles, rhs_tiles, k_tiles, wbits, abits,
    wsigned, asigned, wbase, abase, idsc.accel_buf_in, idsc.accel_buf_out
  );
  registry.push_back(idsc);
  // instruction generation for the rest of the execution is done dynamically
  // in the execLayer calls for now
  return ret;
}
// execute layer with given handle
// in and out are assumed to be preallocated to appropriate buffer sizes,
// depending on the type of layer
void execMatMulLayer(LayerHandle id, const uint8_t * in, int32_t * out) {
  BISMORT_DEBUG("[execMatMulLayer] id " << id);
  const InternalLayerDescriptor dsc = registry[id];
  const gemmbitserial::BitSerialMatrix lhs = dsc.hw_ctx.lhs;
  gemmbitserial::BitSerialMatrix rhs = dsc.hw_ctx.rhs;
  uint32_t rptr = dsc.accel_buf_out;
  BISMORT_DEBUG("lhs_tiles " << lhs_tiles << " rhs_tiles " << rhs_tiles);
  BISMORT_DEBUG("wbase " << wbase << " abase " << abase);
  BISMORT_DEBUG("wbits " << wbits << " abits " << abits);
  auto start_time = std::chrono::high_resolution_clock::now();
  auto end_time = std::chrono::high_resolution_clock::now();
  // TODO this needs to be checked for fetch width != exec width
  // (see exec_to_fetch_width_ratio in BitSerialMatMulExecutor)
  assert(cfg.dpaDimCommon == cfg.readChanWidth);
  const size_t k_tiles = lhs.ncols_a / cfg.dpaDimCommon;
  // convert activations into bit-serial format
  p2s(in, dsc.accel_buf_in, rhs);
  // enable all stages
  acc->set_stage_enables(1, 1, 1);
  start_time = std::chrono::high_resolution_clock::now();
  for (auto & instr : dsc.instructions_queue)
  {
    acc->pushInstruction(instr);
  }
  // wait until all writes are completed
  while(acc->res_opcount() != 0) {
    BISMORT_DEBUG("[execMatMulLayer] waiting for exec, ops f/e/r: " << acc->fetch_opcount() << " " << acc->exec_opcount() << " " << acc->res_opcount());
  };

  end_time = std::chrono::high_resolution_clock::now();
  auto exec_duration_time = std::chrono::duration_cast<std::chrono::microseconds>(end_time-start_time).count();
  BISMORT_DEBUG("[execMatMulLayer] Execution Time: " << exec_duration_time << " us" );
  // copy result buffer to host
  if((lhs.nrows_a == dsc.ctx.lhs.nrows) && (rhs.nrows_a == dsc.ctx.rhs.nrows)) {
    // no padding was necessary, copy directly to host
    platform->copyBufferAccelToHost((void *)dsc.accel_buf_out, (void *)out, dsc.nbytes_buf_out);
  } else {
    // accelerator computed a padded result matrix, copy actual parts only
    AccumType * acc_buf_out = (AccumType *) dsc.accel_buf_out;
    size_t nbytes_row_nonpadded = sizeof(AccumType) * dsc.ctx.lhs.nrows;
    for(size_t row = 0; row < dsc.ctx.rhs.nrows; row++) {
      size_t ind_padded = row * lhs.nrows_a;
      size_t ind_actual = row * dsc.ctx.lhs.nrows;
      platform->copyBufferAccelToHost(&acc_buf_out[ind_padded], (void *)(&out[ind_actual]), nbytes_row_nonpadded);
    }
  }

#ifdef BISMORT_MATMUL_VERIFY_AGAINST_CPU
  // compute result with CPU and compare
  size_t actual_res_bytes = sizeof(AccumType) * dsc.ctx.lhs.nrows * dsc.ctx.rhs.nrows;
  dsc.ctx.rhs.importRegular(in);
  gemmbitserial::gemmBitSerial(dsc.ctx);
  int ret = memcmp(dsc.ctx.res, out, actual_res_bytes);
  cout << "memcmp against golden = " << ret << endl;
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
