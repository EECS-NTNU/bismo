#include "bismo_inference_internal.hpp"

namespace bismo_inference {
// parameter shape: weights[ofm][ifm][ksize][ksize]
LayerHandle initConvLayer(ConvLayerDescriptor & dsc, const uint8_t * weights) {
  gemmbitserial::ConvBitSerialContext ctx = acc->allocConvBitSerialContext(
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
  // dependency on PackedBitGroupType here
  assert(cfg.dpaDimCommon == 64);
  size_t wbytes = lhs.wordsPerBitplane() * lhs.nbits * sizeof(PackedBitGroupType);
  // the rhs for the hardware depends on who is doing the lowering
  size_t abytes_lowered = rhs.wordsPerBitplane() * rhs.nbits * sizeof(PackedBitGroupType);
  size_t abytes_nonlowered = abuf.wordsPerBitplane() * abuf.nbits * sizeof(PackedBitGroupType);
  size_t abytes = dsc.useCPULowering ? abytes_lowered : abytes_nonlowered;

  size_t resbytes = lhs.nrows_a * rhs.nrows_a * sizeof(AccumType);
  uint32_t wbase = allocWeightOCM(wbytes);
  uint32_t abase = allocActivationOCM(abytes);

  // importWeights uses uint64_t here, so Dk must be 64 unless fixed
  assert(cfg.dpaDimCommon == 64);
  ctx.importWeights(weights);
  // TODO a lot of common code between matmul and conv here, should move to a
  // common function call
  // allocate DRAM buffer and copy bit-serial weights there
  // TODO optimization: can use a single DRAM buffer whose size is the largest
  // weight buffer since this is done only once per layer
  uint32_t accel_lhs_ptr = (uint32_t)(uint64_t)platform->allocAccelBuffer(wbytes);
  platform->copyBufferHostToAccel((void *)lhs.data, (void *)accel_lhs_ptr, wbytes);
  // create instruction sequence to fetch weights into OCM
  // TODO this needs to be checked for fetch width != exec width
  // (see exec_to_fetch_width_ratio in BitSerialMatMulExecutor)
  assert(cfg.dpaDimCommon == cfg.readChanWidth);

  std::vector<BISMOInstruction> instrFetchWeights;
  genFetchInstrs(instrFetchWeights, wbase, true, accel_lhs_ptr, lhs.ncols_a / cfg.dpaDimCommon, wbytes);
  acc->set_stage_enables(0, 0, 0);
  for(auto & fi : instrFetchWeights) {
    acc->pushInstruction(fi);
  }
  BISMORT_DEBUG("[initConvLayer] created weight init instruction");
  // launch weight fetch and wait until complete
  BISMORT_DEBUG("[initConvLayer] waiting for weight init, ops f/e/r: " << acc->fetch_opcount() << " " << acc->exec_opcount() << " " << acc->res_opcount());
  acc->set_stage_enables(1, 0, 0);
  while(acc->fetch_opcount() != 0) {};
  acc->set_stage_enables(0, 0, 0);
  BISMORT_DEBUG("[initConvLayer] weight init done");
  // create entry in layer registry and return layer handle
  InternalLayerDescriptor idsc;
  idsc.layerType = layerConv;
  idsc.cnv_ctx = ctx;
  idsc.ctx = ctx.gemmctx;
  idsc.nbytes_buf_in = abytes;
  idsc.nbytes_buf_out = lhs.nrows_a * rhs.nrows_a * sizeof(AccumType) ;
  idsc.cnv_dsc = dsc;
  idsc.wbase = wbase;
  idsc.abase = abase;
  idsc.accel_buf_in = (uint32_t)(uint64_t)platform->allocAccelBuffer(abytes);
  BISMORT_DEBUG("[initConvLayer] accel_buf_in: " << (idsc.accel_buf_in) << " for " << abytes << " bytes");
  // TODO optimization: can use common buffer for all results, since 1 layer
  // at a time
  idsc.accel_buf_out = (uint32_t)(uint64_t)platform->allocAccelBuffer(resbytes);
  BISMORT_DEBUG("[initConvLayer] accel_buf_out: " << (idsc.accel_buf_out) << " for " << resbytes << " bytes");
  LayerHandle ret = registry.size();
  BISMORT_DEBUG("[initConvLayer] registered new conv layer with id " << ret);
  /********************** INSTRUCTION GENERATION ************************/
  // TODO fixes hardware lowering: everything concerning rhs here
  assert(dsc.useCPULowering);
  const size_t lhs_tiles = lhs.nrows_a /  cfg.dpaDimLHS;
  const size_t rhs_tiles = rhs.nrows_a /  cfg.dpaDimRHS;
  const size_t wbits = lhs.nbits;
  const size_t abits = rhs.nbits;
  uint32_t rptr = idsc.accel_buf_out;
  assert(cfg.dpaDimCommon == cfg.readChanWidth);
  const size_t k_tiles = lhs.ncols_a / cfg.dpaDimCommon;
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
  idsc.instructions_queue.push_back(sync.asRaw());
  // generate fetch for activations
  genFetchInstrs(idsc.instructions_queue, abase, false, idsc.accel_buf_in, lhs.ncols_a / cfg.dpaDimCommon, idsc.nbytes_buf_in);
  // fetch sends token to execute stage
  sync.targetStage = stgFetch;
  sync.isSendToken = 1;
  sync.chanID = 0;
  // acc->pushInstruction(sync.asRaw());
  idsc.instructions_queue.push_back(sync.asRaw());
  //acc->set_stage_enables(0, 0, 0);
  // exec receives token from fetch stage
  sync.targetStage = stgExec;
  sync.isSendToken = 0;
  sync.chanID = 0;
  // acc->pushInstruction(sync.asRaw());
  idsc.instructions_queue.push_back(sync.asRaw());
  // /\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\
  // TODO current code still assumes CPU lowering -- move to SWU!  //
  // /\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\

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
      idsc.instructions_queue.push_back(sync.asRaw());
      for(size_t wbit = 0; wbit < wbits; wbit++) {
        for(size_t abit = 0; abit < abits; abit++) {
          // generate exec instr for current bit position
          const bool tile_first = (wbit == 0) && (abit == 0);
          const bool lbit_last = (wbit == wbits-1);
          const bool rbit_last = (abit == abits-1);
          const bool tile_last = lbit_last && rbit_last;
          const bool neg_l = lbit_last && lhs.issigned;
          const bool neg_r = rbit_last && rhs.issigned;
          bool isNeg = neg_l ^ neg_r;
          erc.lhsOffset = wbase + k_tiles * (lhs_tile + wbit * lhs_tiles);
          erc.rhsOffset = abase + k_tiles * (rhs_tile + abit * rhs_tiles);
          erc.negate = isNeg ? 1 : 0;
          erc.numTiles = k_tiles;
          erc.shiftAmount = (wbit + abit);
          erc.clear_before_first_accumulation = tile_first ? 1 : 0;
          erc.writeEn = tile_last ? 1 : 0;
          erc.cnvAddrGenMode = 0;
          if(tile_last) {
            // TODO more flexible multi-buffering here
            erc.writeAddr = (erc.writeAddr == 0) ? 1 : 0;
          }
          // acc->pushInstruction(erc.asRaw());
          idsc.instructions_queue.push_back(erc.asRaw());

        }
      }
      // exec sends token to result stage (send full res buffer)
      sync.targetStage = stgExec;
      sync.isSendToken = 1;
      sync.chanID = 1;
      // acc->pushInstruction(sync.asRaw());
      idsc.instructions_queue.push_back(sync.asRaw());

      // result stage =============================================
      // res receives token from exec stage (acquire full res buffer)
      sync.targetStage = stgResult;
      sync.isSendToken = 0;
      sync.chanID = 0;
      // acc->pushInstruction(sync.asRaw());
      idsc.instructions_queue.push_back(sync.asRaw());
      // generate write instruction
      uint32_t lhs_ind = cfg.dpaDimLHS * lhs_tile;
      uint32_t rhs_ind = cfg.dpaDimRHS * rhs_tile;
      size_t ind = rhs_ind * lhs.nrows_a + lhs_ind;
      rrc.dram_base = (rptr + (ind * sizeof(AccumType)));
      rrc.dram_skip = lhs.nrows_a * sizeof(AccumType);
      rrc.waitCompleteBytes = 0;
      rrc.resmem_addr = (rrc.resmem_addr == 0) ? 1 : 0;
      rrc.nop = 0;
      // acc->pushInstruction(rrc.asRaw());
      idsc.instructions_queue.push_back(rrc.asRaw());
      // res sends token to exec stage (send empty res buffer)
      sync.targetStage = stgResult;
      sync.isSendToken = 1;
      sync.chanID = 0;
      // acc->pushInstruction(sync.asRaw());
      idsc.instructions_queue.push_back(sync.asRaw());
    }
  }

  // exec sends token to fetch stage
  sync.targetStage = stgExec;
  sync.isSendToken = 1;
  sync.chanID = 0;
  // acc->pushInstruction(sync.asRaw());
  idsc.instructions_queue.push_back(sync.asRaw());
  // final result instruction to ensure all writes completed
  rrc.nop = 1;
  rrc.waitCompleteBytes = 1;
  rrc.dram_base = 0;
  rrc.dram_skip = 0;
  rrc.resmem_addr = 0;
  // acc->pushInstruction(rrc.asRaw());
  idsc.instructions_queue.push_back(rrc.asRaw());
  auto end_time = std::chrono::high_resolution_clock::now();
  auto instr_gen_duration_time = std::chrono::duration_cast<std::chrono::microseconds>(end_time-start_time).count();
  BISMORT_DEBUG("[initConvLayer] Instruction generation time: " << instr_gen_duration_time << " us" );

  /********************** END of INSTRUCTION GENERATION *****************/
  registry.push_back(idsc);
  // instruction generation for the rest of the execution is done dynamically
  // in the execLayer calls for now
  return ret;
}


void execConvLayer(LayerHandle id, const uint8_t * in, int32_t * out) {
  BISMORT_DEBUG("[execConvLayer] id " << id);
  const InternalLayerDescriptor dsc = registry[id];
  // NOTE: lhs and rhs are swapped, see note in initConvLayer
  const gemmbitserial::BitSerialMatrix lhs = dsc.ctx.rhs;
  gemmbitserial::BitSerialMatrix rhs = dsc.ctx.lhs;
  gemmbitserial::ConvBitSerialContext ctx = dsc.cnv_ctx;

  // using CPU lowering for now
  // TODO switch to hardware SWU
  // TODO fixes hardware lowering: everything concerning rhs here
  assert(dsc.useCPULowering);
  auto start_time = std::chrono::high_resolution_clock::now();
  ctx.importActivations(in);
  auto end_time = std::chrono::high_resolution_clock::now();
  auto rhs2bs_duration_time = std::chrono::duration_cast<std::chrono::microseconds>(end_time-start_time).count();
  BISMORT_DEBUG("[execConvLayer] software p2s+im2row time: " << rhs2bs_duration_time << " us" );

  //  copy the lowered bit-serial activations to the accelerator
  platform->copyBufferHostToAccel(rhs.data, (void *)dsc.accel_buf_in, dsc.nbytes_buf_in);

  // enable all stages
  acc->set_stage_enables(1, 1, 1);
  start_time = std::chrono::high_resolution_clock::now();
  for (auto & instr : dsc.instructions_queue)
  {
    acc->pushInstruction(instr);
  }
  // wait until all writes are completed
  while(acc->res_opcount() != 0) {
    BISMORT_DEBUG("[execConvLayer] waiting for exec, ops f/e/r: " << acc->fetch_opcount() << " " << acc->exec_opcount() << " " << acc->res_opcount());
  };

  end_time = std::chrono::high_resolution_clock::now();
  auto exec_duration_time = std::chrono::duration_cast<std::chrono::microseconds>(end_time-start_time).count();
  BISMORT_DEBUG("[execConvLayer] Execution Time: " << exec_duration_time << " us" );
  // temporary buffer to help with transposition
  AccumType * transpose_tmp = new AccumType[lhs.nrows * rhs.nrows];
  // copy result buffer from accel to host
  //AccumType * targetHostBuf = out;
  AccumType * targetHostBuf = transpose_tmp;
  if((lhs.nrows_a == lhs.nrows) && (rhs.nrows_a == rhs.nrows)) {
    // no padding was necessary, copy directly to host
    platform->copyBufferAccelToHost((void *)dsc.accel_buf_out, (void *)targetHostBuf, dsc.nbytes_buf_out);
  } else {
    // accelerator computed a padded result matrix, copy actual parts only
    AccumType * acc_buf_out = (AccumType *) dsc.accel_buf_out;
    size_t nbytes_row_nonpadded = sizeof(AccumType) * lhs.nrows;
    for(size_t row = 0; row < rhs.nrows; row++) {
      size_t ind_padded = row * lhs.nrows_a;
      size_t ind_actual = row * lhs.nrows;
      platform->copyBufferAccelToHost(&acc_buf_out[ind_padded], (void *)(&targetHostBuf[ind_actual]), nbytes_row_nonpadded);
    }
  }
  // host-to-host transpose
  for(size_t i = 0; i < rhs.nrows; i++) {
    for(size_t j = 0; j < lhs.nrows; j++) {
      out[j * rhs.nrows + i] = transpose_tmp[i * lhs.nrows + j];
    }
  }
  delete [] transpose_tmp;

#ifdef BISMORT_CONV_VERIFY_AGAINST_CPU
  // compute result with CPU and compare
  size_t actual_res_bytes = sizeof(AccumType) * lhs.nrows * rhs.nrows;
  gemmbitserial::gemmBitSerial(ctx.gemmctx);
  int ret = memcmp(ctx.gemmctx.res, out, actual_res_bytes);
  cout << "memcmp against golden = " << ret << endl;
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
