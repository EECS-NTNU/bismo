#include "bismo_inference_internal.hpp"

namespace bismo_inference {

TIMER_INIT();

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
  // convert weights to bit serial
  // don't really care about performance here since this is one-off
  ctx.lhs.importRegular(weights);
  size_t abytes_workload_total = ctx.rhs.wordsPerBitplane() * ctx.rhs.nbits * sizeof(PackedBitGroupType);
  size_t n_act_partitions = getNumPartitionsForActivationOCM(abytes_workload_total);
  gemmbitserial::GEMMContext hw_ctx;
  if(n_act_partitions > 1) {
    // must have room for at least one tile per bit position
    assert(activationOCMBytesLeft >= (ctx.rhs.nbits * ctx.rhs.wordsPerRow() * cfg.dpaDimRHS * sizeof(PackedBitGroupType)));
    // how many rhs tiles from all bit positions fit into the RHS OCM?
    const size_t partition_rhs_ntiles = activationOCMBytesLeft / (ctx.rhs.nbits * ctx.rhs.wordsPerRow() * cfg.dpaDimRHS * sizeof(PackedBitGroupType));
    // allocate hw context for a single partition
    hw_ctx = acc->allocGEMMContext(
      dsc.M, dsc.K, partition_rhs_ntiles * cfg.dpaDimRHS, dsc.wbits, dsc.ibits, dsc.wsigned, dsc.isigned
    );
    // make sure we have enough partitions to cover all of rhs
    while(hw_ctx.rhs.nrows_a * n_act_partitions < ctx.rhs.nrows) {
      n_act_partitions++;
    }
    BISMORT_DEBUG("[initMatMulLayer] RHS partition:");
#ifdef DEBUG
    hw_ctx.rhs.printSummary();
#endif
    // import weights separately
    hw_ctx.lhs.importRegular(weights);
  } else {
    // single RHS partition only
    hw_ctx = ctx;
  }
  // TODO this needs to be checked for fetch width != exec width
  // (see exec_to_fetch_width_ratio in BitSerialMatMulExecutor)
  assert(cfg.dpaDimCommon == cfg.readChanWidth);
  size_t wbytes = hw_ctx.lhs.wordsPerBitplane() * hw_ctx.lhs.nbits * sizeof(PackedBitGroupType);
  size_t abytes = hw_ctx.rhs.wordsPerBitplane() * hw_ctx.rhs.nbits * sizeof(PackedBitGroupType);
  size_t resbytes = hw_ctx.lhs.nrows_a * hw_ctx.rhs.nrows_a * sizeof(AccumType);
  BISMORT_DEBUG("rhs n_act_partitions: " << n_act_partitions << " partition abytes: " << abytes << " available " << activationOCMBytesLeft);
  assert(abytes <= activationOCMBytesLeft);
  uint32_t wbase = allocWeightOCM(wbytes);
  uint32_t abase = 0; // all activations use the same OCM buffer
  // create entry in layer registry and return layer handle
  InternalLayerDescriptor idsc;
  idsc.layerType = layerMatMul;
  idsc.ctx = ctx;
  idsc.hw_ctx = hw_ctx;
  idsc.nbytes_buf_in = abytes;
  idsc.nbytes_buf_out = hw_ctx.lhs.nrows_a * hw_ctx.rhs.nrows_a * sizeof(AccumType) ;
  idsc.padded_result_host_buffer = new AccumType[hw_ctx.lhs.nrows_a * hw_ctx.rhs.nrows_a * n_act_partitions];
  idsc.mm_dsc = dsc;
  idsc.wbase = wbase;
  idsc.wbytes = wbytes;
  idsc.accel_lhs_ptr = (uint32_t)(uint64_t)platform->allocAccelBuffer(wbytes);
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
#ifdef BISMORT_USE_INSTRGEN
  // TODO handle even distribution of rows to mems with instrgen alignment
  // TODO enable multiple fetch instrs for larger chunks
  assert(abytes <= FETCH_BLOCK_MAX);
  // create an instruction generation descriptor
  SingleMMDescriptor instrgen_dsc;
  instrgen_dsc.tiles_m = lhs_tiles;
  instrgen_dsc.tiles_k = k_tiles;
  instrgen_dsc.tiles_n = rhs_tiles;
  instrgen_dsc.bits_l = wbits;
  instrgen_dsc.bits_r = abits;
  instrgen_dsc.signed_l = wsigned;
  instrgen_dsc.signed_r = asigned;
  instrgen_dsc.base_l = wbase;
  instrgen_dsc.base_r = abase;
  instrgen_dsc.base_res = 0;
  instrgen_dsc.nbufs_res = 2;
  // LHS already fetched, skip
  instrgen_dsc.dram_lhs = 0xffffffff;
  instrgen_dsc.dram_rhs = idsc.accel_buf_in;
  instrgen_dsc.dram_res = idsc.accel_buf_out;
  idsc.instrgen_dsc = instrgen_dsc;
#else
  // generate instruction sequence on the CPU
  genMatMulInstrs_LHSPreloaded_RHSFitsOnChip(
    idsc.instructions_queue, lhs_tiles, rhs_tiles, k_tiles, wbits, abits,
    wsigned, asigned, wbase, abase, idsc.accel_buf_in, idsc.accel_buf_out
  );
#endif
  registry.push_back(idsc);
  // set the contents of the LHS matrix for the accelerator
  configMatMulLayer_Internal_SetLHS(ret, hw_ctx.lhs);
  return ret;
}

// replace the LHS matrix for a layer
// specified matrix shape and initialized matrix shape must be the same
void configMatMulLayer_Internal_SetLHS(LayerHandle id, gemmbitserial::BitSerialMatrix mat) {
  TIMER_SAMPLE();
  // instrgen doesn't support only fetching LHS, use manual feed
  acc->useDirectInstructionFeed();
  InternalLayerDescriptor dsc = registry[id];
  // allocate DRAM buffer and copy bit-serial weights there
  // TODO optimization: can use a single DRAM buffer whose size is the largest
  // weight buffer since this is done only once per layer
  uint32_t accel_lhs_ptr = dsc.accel_lhs_ptr;
  const size_t wbytes = dsc.wbytes;
  const size_t wbase = dsc.wbase;
#ifdef DEBUG
  BISMORT_DEBUG("[configMatMulLayer_Internal_SetLHS] Source matrix:");
  mat.printSummary();
  BISMORT_DEBUG("[configMatMulLayer_Internal_SetLHS] HW shape matrix:");
  dsc.ctx.lhs.printSummary();
#endif
  // ensure shapes are compatible
  assert(mat.nbits == dsc.ctx.lhs.nbits);
  assert(mat.wordsPerBitplane() == dsc.ctx.lhs.wordsPerBitplane());
  platform->copyBufferHostToAccel((void *)mat.data, (void *)accel_lhs_ptr, wbytes);
  // if bit serial matrix data is coming from an external source, copy to
  // internal as well for debug purposes
  if(dsc.ctx.lhs.data != mat.data) {
    memcpy(dsc.ctx.lhs.data, mat.data, wbytes);
  }
  // create instruction sequence to fetch weights into OCM
  std::vector<BISMOInstruction> instrFetchWeights;
  genFetchInstrs(instrFetchWeights, wbase, true, accel_lhs_ptr, mat.ncols_a / cfg.dpaDimCommon, wbytes);
  acc->set_stage_enables(0, 0, 0);
  for(auto & fi : instrFetchWeights) {
    acc->pushInstruction(fi);
  }
  BISMORT_DEBUG("[configMatMulLayer_Internal_SetLHS] created weight init instructions: " << acc->fetch_opcount());
  // launch weight fetch and wait until complete
  acc->set_stage_enables(1, 0, 0);
  while(acc->fetch_opcount() != 0) {
    //BISMORT_DEBUG("[initMatMulLayer] waiting for weight init, ops f/e/r: " << acc->fetch_opcount() << " " << acc->exec_opcount() << " " << acc->res_opcount());
  };
  acc->set_stage_enables(0, 0, 0);
  BISMORT_DEBUG("[configMatMulLayer_Internal_SetLHS] weight init done");
  TIMER_SAMPLE();
  TIMER_REPORT("configMatMulLayer_Internal_SetLHS total");
}

// execute layer with given handle
// in and out are assumed to be preallocated to appropriate buffer sizes,
// depending on the type of layer
void execMatMulLayer(LayerHandle id, const uint8_t * in, int32_t * out) {
  BISMORT_DEBUG("[execMatMulLayer] id " << id);
  InternalLayerDescriptor dsc = registry[id];
  gemmbitserial::BitSerialMatrix lhs = dsc.hw_ctx.lhs;
  gemmbitserial::BitSerialMatrix rhs = dsc.hw_ctx.rhs;
  dsc.ctx.rhs.importRegular(in);
  execMatMulLayer_Internal_RHSBitSerial(id, out);
}

// asssumes that the RHS matrix is already in bit-serial format in the ctx
void execMatMulLayer_Internal_RHSBitSerial(LayerHandle id, int32_t * out) {
  BISMORT_DEBUG("[execMatMulLayer_Internal_RHSBitSerial] id " << id);
  InternalLayerDescriptor dsc = registry[id];
  gemmbitserial::BitSerialMatrix lhs = dsc.hw_ctx.lhs;
  gemmbitserial::BitSerialMatrix rhs = dsc.hw_ctx.rhs;
  AccumType * padded_result_host_buffer = dsc.padded_result_host_buffer;
  uint32_t rptr = dsc.accel_buf_out;
  for(size_t rhs_p = 0; rhs_p < dsc.n_act_partitions; rhs_p++) {
    TIMER_SAMPLE();
    // calculate start of rhs partition. each partition has hw_ctx.rhs.nrows_a
    // rows, and the regular amount of columns for the unpadded bit-parallel matrix
    size_t rhs_partition_start_row = rhs.nrows_a * rhs_p;
    BISMORT_DEBUG("Processing RHS partition " << rhs_p << " row " << rhs_partition_start_row);
    // convert activations into bit-serial format
    size_t host_rows_in_partition = rhs.nrows;
    if(rhs_partition_start_row + host_rows_in_partition > dsc.ctx.rhs.nrows) {
      BISMORT_DEBUG("Adjusting rhs.nrows to prevent reading past end of host buffer: ");
      BISMORT_DEBUG("Was " << rhs.nrows << " now " << dsc.ctx.rhs.nrows - rhs_partition_start_row);
      // prevent reading past the end of host buffer
      host_rows_in_partition = dsc.ctx.rhs.nrows - rhs_partition_start_row;
    }
    // copy the partition from each bitslice into the accelerator memory
    // loop over each bitplane
    const size_t bytes_per_bitplane_part_host = host_rows_in_partition * rhs.wordsPerRow() * sizeof(uint64_t);
    const size_t bytes_per_bitplane_part_accel = rhs.nrows_a * rhs.wordsPerRow() * sizeof(uint64_t);
    for(size_t b = 0; b < rhs.nbits; b++) {
      uint64_t * host_ptr = dsc.ctx.rhs.rowptr(b, rhs_partition_start_row);
      uint32_t accel_ptr = dsc.accel_buf_in + b * bytes_per_bitplane_part_accel;
      platform->copyBufferHostToAccel(host_ptr, (void *)accel_ptr, bytes_per_bitplane_part_host);
    }
    TIMER_SAMPLE();
    TIMER_REPORT("execMatMulLayer host->accel copy");

    // enable all stages
    acc->set_stage_enables(1, 1, 1);
#ifdef BISMORT_USE_INSTRGEN
    acc->useDescriptors();
    // feed the instrgen descriptor
    acc->pushSingleMMDescriptor(dsc.instrgen_dsc);
    // HACK: make sure at least one op has appeared before checking for completion
    // proper way to fix this is to singal completion from accel explicitly
    while(acc->res_opcount() == 0) {};
#else
    acc->useDirectInstructionFeed();
    for (auto & instr : dsc.instructions_queue) {
      acc->pushInstruction(instr);
    }
#endif
    // wait until all writes are completed
    while(acc->res_opcount() != 0) {
      //BISMORT_DEBUG("[execMatMulLayer] waiting for exec, ops f/e/r: " << acc->fetch_opcount() << " " << acc->exec_opcount() << " " << acc->res_opcount());
    };
    TIMER_SAMPLE();
    TIMER_REPORT("execMatMulLayer hardware execution");
    // copy padded result buffer to host
    size_t result_partition_start_elem = rhs_partition_start_row * lhs.nrows_a;
    platform->copyBufferAccelToHost((void *)dsc.accel_buf_out, (void *) &padded_result_host_buffer[result_partition_start_elem], dsc.nbytes_buf_out);
    TIMER_SAMPLE();
    TIMER_REPORT("execMatMulLayer accel->host copy");
  }
  // get rid of padding as needed
  if((lhs.nrows_a == dsc.ctx.lhs.nrows) && (dsc.n_act_partitions * rhs.nrows_a == dsc.ctx.rhs.nrows)) {
    // no padding was necessary, copy directly
    memcpy(out, padded_result_host_buffer, lhs.nrows_a * rhs.nrows_a * dsc.n_act_partitions * sizeof(AccumType));
  } else {
    // accelerator computed a padded result matrix, copy actual parts only
    size_t nbytes_row_nonpadded = sizeof(AccumType) * dsc.ctx.lhs.nrows;
    for(size_t row = 0; row < dsc.ctx.rhs.nrows; row++) {
      size_t ind_padded = row * lhs.nrows_a;
      size_t ind_actual = row * dsc.ctx.lhs.nrows;
      memcpy((void *)(&out[ind_actual]), &padded_result_host_buffer[ind_padded], nbytes_row_nonpadded);
    }
  }
  TIMER_SAMPLE();
  TIMER_REPORT("execMatMulLayer remove padding");

#ifdef BISMORT_MATMUL_VERIFY_AGAINST_CPU
  // compute result with CPU and compare
  size_t actual_res_bytes = sizeof(AccumType) * dsc.ctx.lhs.nrows * dsc.ctx.rhs.nrows;
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
