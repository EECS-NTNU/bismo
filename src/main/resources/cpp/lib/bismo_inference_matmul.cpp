#include "bismo_inference_internal.hpp"

namespace bismo_inference {

void genMatMulInstrs_LHSPreloaded_RHSFitsOnChip(
  std::vector<BISMOInstruction> & ins,
  size_t lhs_tiles, size_t rhs_tiles, size_t k_tiles,
  size_t wbits, size_t abits,
  bool wsigned, bool asigned,
  size_t wbase_ocm, size_t abase_ocm,
  uint32_t accel_buf_rhs,
  uint32_t accel_buf_res
) {
  assert(cfg.dpaDimCommon == cfg.readChanWidth);
  const size_t lhs_nrows_a = lhs_tiles * cfg.dpaDimLHS;
  auto start_time = std::chrono::high_resolution_clock::now();
  // fetch the rhs matrix into the on-chip buffer
  BISMOSyncInstruction sync;
  sync.isRunCfg = 0;
  BISMOExecRunInstruction erc;
  erc.isRunCfg = 1;
  erc.targetStage = stgExec;
  erc.writeAddr = 0;
  BISMOResultRunInstruction rrc;
  rrc.isRunCfg = 1;
  rrc.targetStage = stgResult;
  rrc.resmem_addr = 0;
  // fetch receive token from exec stage for buffer access
  // in the current schedule this token represents the entire RHS buffer
  sync.targetStage = stgFetch;
  sync.isSendToken = 0;
  sync.chanID = 0;
  // acc->pushInstruction(sync.asRaw());
  ins.push_back(sync.asRaw());
  // generate fetch instructions for activations
  const size_t bytes_per_rhs_tile = (cfg.dpaDimCommon * cfg.dpaDimRHS) / 8;
  const size_t rhs_bytes = (bytes_per_rhs_tile * abits * rhs_tiles * k_tiles);
  genFetchInstrs(ins, abase_ocm, false, accel_buf_rhs, k_tiles, rhs_bytes);
  // fetch sends token to execute stage
  sync.targetStage = stgFetch;
  sync.isSendToken = 1;
  sync.chanID = 0;
  // acc->pushInstruction(sync.asRaw());
  ins.push_back(sync.asRaw());
  //acc->set_stage_enables(0, 0, 0);
  // exec receives token from fetch stage
  sync.targetStage = stgExec;
  sync.isSendToken = 0;
  sync.chanID = 0;
  // acc->pushInstruction(sync.asRaw());
  ins.push_back(sync.asRaw());
  // TODO this assumes activations fit into OCM, need extra tiling otherwise
  // TODO optimization: do this in smaller chunks for fetch-exec concurrency
  for(size_t lhs_tile = 0; lhs_tile < lhs_tiles; lhs_tile++) {
    for(size_t rhs_tile = 0; rhs_tile < rhs_tiles; rhs_tile++) {
      // exec stage ==============================================
      // exec receives token from result stage (acquire empty res buffer)
      sync.targetStage = stgExec;
      sync.isSendToken = 0;
      sync.chanID = 1;
      // acc->pushInstruction(sync.asRaw());
      ins.push_back(sync.asRaw());
      for(size_t wbit = 0; wbit < wbits; wbit++) {
        for(size_t abit = 0; abit < abits; abit++) {
          // generate exec instr for current bit position
          const bool tile_first = (wbit == 0) && (abit == 0);
          const bool lbit_last = (wbit == wbits-1);
          const bool rbit_last = (abit == abits-1);
          const bool tile_last = lbit_last && rbit_last;
          const bool neg_l = lbit_last && wsigned;
          const bool neg_r = rbit_last && asigned;
          bool isNeg = neg_l ^ neg_r;
          erc.lhsOffset = wbase_ocm + k_tiles * (lhs_tile + wbit * lhs_tiles);
          erc.rhsOffset = abase_ocm + k_tiles * (rhs_tile + abit * rhs_tiles);
          erc.negate = isNeg ? 1 : 0;
          erc.numTiles = k_tiles;
          erc.shiftAmount = (wbit + abit);
          erc.clear_before_first_accumulation = tile_first ? 1 : 0;
          erc.writeEn = tile_last ? 1 : 0;
          if(tile_last) {
            // TODO more flexible multi-buffering here
            erc.writeAddr = (erc.writeAddr == 0) ? 1 : 0;
          }
          // acc->pushInstruction(erc.asRaw());
          ins.push_back(erc.asRaw());
        }
      }
      // exec sends token to result stage (send full res buffer)
      sync.targetStage = stgExec;
      sync.isSendToken = 1;
      sync.chanID = 1;
      ins.push_back(sync.asRaw());

      // result stage =============================================
      // res receives token from exec stage (acquire full res buffer)
      sync.targetStage = stgResult;
      sync.isSendToken = 0;
      sync.chanID = 0;
      ins.push_back(sync.asRaw());
      // generate write instruction
      uint32_t lhs_ind = cfg.dpaDimLHS * lhs_tile;
      uint32_t rhs_ind = cfg.dpaDimRHS * rhs_tile;
      size_t ind = rhs_ind * lhs_nrows_a + lhs_ind;
      rrc.dram_base = (accel_buf_res + (ind * sizeof(AccumType)));
      rrc.dram_skip = lhs_nrows_a * sizeof(AccumType);
      rrc.waitCompleteBytes = 0;
      rrc.resmem_addr = (rrc.resmem_addr == 0) ? 1 : 0;
      rrc.nop = 0;
      ins.push_back(rrc.asRaw());
      // res sends token to exec stage (send empty res buffer)
      sync.targetStage = stgResult;
      sync.isSendToken = 1;
      sync.chanID = 0;
      ins.push_back(sync.asRaw());
    }
  }
  // exec sends token to fetch stage
  sync.targetStage = stgExec;
  sync.isSendToken = 1;
  sync.chanID = 0;
  ins.push_back(sync.asRaw());
  // final result instruction to ensure all writes completed
  rrc.nop = 1;
  rrc.waitCompleteBytes = 1;
  rrc.dram_base = 0;
  rrc.dram_skip = 0;
  rrc.resmem_addr = 0;
  ins.push_back(rrc.asRaw());
  auto end_time = std::chrono::high_resolution_clock::now();
  auto instr_gen_duration_time = std::chrono::duration_cast<std::chrono::microseconds>(end_time-start_time).count();
  BISMORT_DEBUG("[genMatMulInstrs_LHSPreloaded_RHSFitsOnChip] Instruction generation time: " << instr_gen_duration_time << " us" );
}


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
  size_t wbytes = ctx.lhs.wordsPerBitplane() * ctx.lhs.nbits * sizeof(PackedBitGroupType);
  size_t abytes = ctx.rhs.wordsPerBitplane() * ctx.rhs.nbits * sizeof(PackedBitGroupType);
  size_t resbytes = ctx.lhs.nrows_a * ctx.rhs.nrows_a * sizeof(AccumType);
  uint32_t wbase = allocWeightOCM(wbytes);
  uint32_t abase = 0; // all activations use the same OCM buffer
  size_t n_act_partitions = getNumPartitionsForActivationOCM(abytes);
  // convert weights to bit serial
  // don't really care about performance here since this is one-off
  ctx.lhs.importRegular(weights);
  // allocate DRAM buffer and copy bit-serial weights there
  // TODO optimization: can use a single DRAM buffer whose size is the largest
  // weight buffer since this is done only once per layer
  uint32_t accel_lhs_ptr = (uint32_t)(uint64_t)platform->allocAccelBuffer(wbytes);
  platform->copyBufferHostToAccel((void *)ctx.lhs.data, (void *)accel_lhs_ptr, wbytes);
  // create instruction sequence to fetch weights into OCM
  // TODO this needs to be checked for fetch width != exec width
  // (see exec_to_fetch_width_ratio in BitSerialMatMulExecutor)
  assert(cfg.dpaDimCommon == cfg.readChanWidth);
  std::vector<BISMOInstruction> instrFetchWeights;
  genFetchInstrs(instrFetchWeights, wbase, true, accel_lhs_ptr, ctx.lhs.ncols_a / cfg.dpaDimCommon, wbytes);
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
  idsc.nbytes_buf_in = abytes;
  idsc.nbytes_buf_out = ctx.lhs.nrows_a * ctx.rhs.nrows_a * sizeof(AccumType) ;
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
  const size_t lhs_tiles = ctx.lhs.nrows_a /  cfg.dpaDimLHS;
  const size_t rhs_tiles = ctx.rhs.nrows_a /  cfg.dpaDimRHS;
  const size_t k_tiles = ctx.rhs.ncols_a /  cfg.dpaDimCommon;
  const size_t wbits = ctx.lhs.nbits;
  const size_t abits = ctx.rhs.nbits;
  const bool wsigned = ctx.lhs.issigned;
  const bool asigned = ctx.rhs.issigned;
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
  const gemmbitserial::BitSerialMatrix lhs = dsc.ctx.lhs;
  gemmbitserial::BitSerialMatrix rhs = dsc.ctx.rhs;
  const size_t lhs_tiles = lhs.nrows_a /  cfg.dpaDimLHS;
  const size_t rhs_tiles = rhs.nrows_a /  cfg.dpaDimRHS;
  const size_t wbase = dsc.wbase;
  const size_t abase = dsc.abase;
  const size_t wbits = lhs.nbits;
  const size_t abits = rhs.nbits;
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
  if((lhs.nrows_a == lhs.nrows) && (rhs.nrows_a == rhs.nrows)) {
    // no padding was necessary, copy directly to host
    platform->copyBufferAccelToHost((void *)dsc.accel_buf_out, (void *)out, dsc.nbytes_buf_out);
  } else {
    // accelerator computed a padded result matrix, copy actual parts only
    AccumType * acc_buf_out = (AccumType *) dsc.accel_buf_out;
    size_t nbytes_row_nonpadded = sizeof(AccumType) * lhs.nrows;
    for(size_t row = 0; row < rhs.nrows; row++) {
      size_t ind_padded = row * lhs.nrows_a;
      size_t ind_actual = row * lhs.nrows;
      platform->copyBufferAccelToHost(&acc_buf_out[ind_padded], (void *)(&out[ind_actual]), nbytes_row_nonpadded);
    }
  }

#ifdef BISMORT_MATMUL_VERIFY_AGAINST_CPU
  // compute result with CPU and compare
  size_t actual_res_bytes = sizeof(AccumType) * lhs.nrows * rhs.nrows;
  gemmbitserial::gemmBitSerial(dsc.ctx);
  int ret = memcmp(dsc.ctx.res, out, actual_res_bytes);
  cout << "memcmp against golden = " << ret << endl;
  if(ret != 0) {
    cout << "expected vs found" << endl;
    for(int i = 0; i < lhs.nrows * rhs.nrows; i++) {
      if(dsc.ctx.res[i] != out[i]) {
        cout << "pos " << i << ": " << dsc.ctx.res[i] << " " << out[i] << endl;
      }
    }
  }
#endif
}
}
