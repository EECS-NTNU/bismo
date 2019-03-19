#include "bismo_inference_internal.hpp"

namespace bismo_inference {
TIMER_INIT();
// global handle for the platform and BISMO driver
WrapperRegDriver * platform;
BitSerialMatMulAccelDriver * acc;
HardwareCfg cfg;
uint32_t weightOCMBase, weightOCMBytesLeft;
uint32_t activationOCMBase, activationOCMBytesLeft;
uint32_t thresholdOCMBase, thresholdOCMBytesLeft;
std::vector<InternalLayerDescriptor> registry;
std::map<std::string,float> instrumentationData;

// global init/deinit for the runtime library
void init() {
  platform = initPlatform();
  acc = new BitSerialMatMulAccelDriver(platform);
  acc->reset();
  // currently the runtime is implemented with direct instruction feed
  // will switch to descriptors when the correct generators are impl'd
  acc->init_resource_pools();
  acc->useDirectInstructionFeed();
  cfg = acc->hwcfg();
  // initialize OCM resource pool from hwcfg
  weightOCMBase = 0;
  weightOCMBytesLeft = acc->get_lhs_total_BRAM_bytes();
  activationOCMBase = 0;
  activationOCMBytesLeft = acc->get_rhs_total_BRAM_bytes();
  thresholdOCMBase = 0;
  // TODO set thresholdOCMBytesLeft from hwcfg
}

void deinit() {
  delete acc;
  deinitPlatform(platform);
}

InstrumentationData getInstrumentationData() {
  return instrumentationData;
}

void benchmark_host_accel_transfer() {
  std::vector<size_t> vsize {1, 2, 4, 8, 16, 32};
  for(auto & s : vsize) {
    size_t nbytes = s * 1024;
    uint8_t * hostbuf = new uint8_t[nbytes];
    memset(hostbuf, 0x1f, nbytes);
    void * accelbuf = platform->allocAccelBuffer(nbytes);
    TIMER_SAMPLE();
    platform->copyBufferHostToAccel(hostbuf, accelbuf, nbytes);
    TIMER_SAMPLE();
    TIMER_REPORT("host to accel");
    TIMER_SAMPLE();
    platform->copyBufferAccelToHost(accelbuf, hostbuf, nbytes);
    TIMER_SAMPLE();
    TIMER_REPORT("accel to host");
    delete [] hostbuf;
    platform->deallocAccelBuffer(accelbuf);
  }
}

HardwareConfig getHardwareConfig() {
  HardwareConfig ret;
  ret.accWidth = cfg.accWidth;
  ret.cmdQueueEntries = cfg.cmdQueueEntries;
  ret.dpaDimCommon = cfg.dpaDimCommon;
  ret.dpaDimLHS = cfg.dpaDimLHS;
  ret.dpaDimRHS = cfg.dpaDimRHS;
  ret.lhsEntriesPerMem = cfg.lhsEntriesPerMem;
  ret.maxShiftSteps = cfg.maxShiftSteps;
  ret.readChanWidth = cfg.readChanWidth;
  ret.rhsEntriesPerMem = cfg.rhsEntriesPerMem;
  ret.writeChanWidth = cfg.writeChanWidth;
  return ret;
}

uint32_t allocWeightOCM(size_t nbytes) {
  // check if enough weight OCM is left
  // TODO: if not enough space, fail gracefully to permit SW execution
  BISMORT_DEBUG("[allocWeightOCM] alloc " << nbytes << ", available " << weightOCMBytesLeft);
  assert(nbytes <= weightOCMBytesLeft);
  // increment pointer to next available OCM slot
  weightOCMBytesLeft -= nbytes;
  uint32_t ret = weightOCMBase;
  // convert bytes to LHS mem address
  // TODO this conversion needs to be checked for fetch width != exec width
  assert(cfg.dpaDimCommon == cfg.readChanWidth);
  const size_t bytesPerWeightOCMEntry = (cfg.dpaDimLHS * cfg.dpaDimCommon) / 8;
  weightOCMBase += nbytes / bytesPerWeightOCMEntry;
  // return allocated base address
  return ret;
}

uint32_t allocThresOCM(size_t nbytes) {
  // check if enough threshold OCM is left
  assert(nbytes <= thresholdOCMBytesLeft);
  thresholdOCMBytesLeft -= nbytes;
  // TODO implement allocThresOCM
  // TOOD return allocated base address
  return 0;
}

size_t getNumPartitionsForActivationOCM(size_t nbytes) {
  // since we follow a strictly layer-by-layer strategy where only one
  // layer executes at a time, we simply use the same OCM buffer for all layers
  // see how many partitions we need
  size_t aligned_nbytes = gemmbitserial::alignTo(nbytes, activationOCMBytesLeft);
  size_t n_partitions = aligned_nbytes / activationOCMBytesLeft;
  BISMORT_DEBUG("[getNumPartitionsForActivationOCM] alloc " << nbytes << ", available " << activationOCMBytesLeft);
  BISMORT_DEBUG("[getNumPartitionsForActivationOCM] n_partitions " << n_partitions);
  return n_partitions;
}

void genFetchInstrs(
  std::vector<BISMOInstruction> & ins,
  size_t bram_base,
  bool lhsNotRhs,
  uint32_t dram_base,
  size_t tiles_per_row,
  size_t nbytes
) {
  BISMOFetchRunInstruction frc;
  size_t bram_start = lhsNotRhs ? acc->get_fetch_first_lhs_id() : acc->get_fetch_first_rhs_id();
  size_t bram_range = (lhsNotRhs ? cfg.dpaDimLHS : cfg.dpaDimRHS) - 1;

  frc.isRunCfg = 1;
  frc.targetStage = stgFetch;
  frc.bram_id_start = bram_start;
  frc.bram_id_range = lhsNotRhs ? 0 : 1;
  frc.tiles_per_row = tiles_per_row;
  frc.bram_addr_base = bram_base;
  frc.dram_base = dram_base;
  size_t bytes_left = nbytes;
  const size_t bytes_per_addr = (lhsNotRhs ? cfg.dpaDimLHS : cfg.dpaDimRHS) * (cfg.dpaDimCommon/8);
  // bram base addr calculations here assume that each non-final packet is
  // distributed evenly between memories. need a max block size that is divisible
  // by tiles_per_row * bytes_per_addr for this.
  const size_t aligned_chunks = FETCH_BLOCK_MAX / (tiles_per_row * bytes_per_addr);
  const size_t max_block = aligned_chunks * (tiles_per_row * bytes_per_addr);
  while(bytes_left > 0) {
    frc.dram_block_size_bytes = std::min(max_block, bytes_left);
    frc.dram_block_offset_bytes = frc.dram_block_size_bytes;
    frc.dram_block_count = bytes_left / frc.dram_block_size_bytes;
    ins.push_back(frc.asRaw());
    BISMORT_DEBUG("[genFetchInstrs] " << frc);
    size_t last_chunk_bytes = frc.dram_block_count * frc.dram_block_size_bytes;
    bytes_left -= last_chunk_bytes;
    frc.dram_base += last_chunk_bytes;
    frc.bram_addr_base += last_chunk_bytes / bytes_per_addr;
  }
}

// generate instructions for a matrix multiplication where the LHS matrix
// is already preloaded to onLHS -chip memory, and the RHS matrix is small enough
// to fit into the RHS on-chip memory
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
          // acc->pushInstruction(erc.asRaw());
          ins.push_back(erc.asRaw());
          if(tile_last) {
            // update resmem addr
            erc.writeAddr = (erc.writeAddr == 0) ? 1 : 0;
          }
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
      rrc.nop = 0;
      ins.push_back(rrc.asRaw());
      // update resmem addr
      rrc.resmem_addr = (rrc.resmem_addr == 0) ? 1 : 0;
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

// parameter shape: thresholds[nthresholds][nchannels]
LayerHandle initThresLayer(ThresLayerDescriptor & dsc, const uint8_t * thresholds, bool cpu_only) {
  // TODO allocate OCM space for thresholds
  // TODO write thresholds into OCM
  // TODO create instruction sequence for execution, store for later
  // TODO create entry in layer registry
  // TODO return layer handle
  return 0;
}

// bit-parallel to bit-serial conversion
void p2s(const uint8_t * host_buf, uint32_t accel_buf, gemmbitserial::BitSerialMatrix & mat) {
  size_t nbytes = mat.wordsPerBitplane() * mat.nbits * sizeof(PackedBitGroupType);
  auto start_time = std::chrono::high_resolution_clock::now();
  auto end_time = std::chrono::high_resolution_clock::now();
#ifdef FORCE_SW_P2S
  // force software p2s, useful to rule out hw p2s bugs
  start_time = std::chrono::high_resolution_clock::now();
  mat.importRegular((uint8_t *)host_buf);
  platform->copyBufferHostToAccel(mat.data, (void *) accel_buf, nbytes);
  end_time = std::chrono::high_resolution_clock::now();
  auto rhs2bs_duration_time = std::chrono::duration_cast<std::chrono::microseconds>(end_time-start_time).count();
  BISMORT_DEBUG("[execMatMulLayer] software bit-serialization+copy time: " << rhs2bs_duration_time << " us" );
#else
  // use hardware p2s
  // Create a tmp in buff for parallel activations
  // TODO this is needed only once per network (allocate for largest activation),
  // infrastructure functions (alloc etc) should not be called during inference
  size_t nbytes_bitpar = mat.nrows_a * mat.ncols_a * sizeof(uint8_t);
  void * inbuf_p2s = platform->allocAccelBuffer(nbytes_bitpar);
#ifdef P2S_CLEAR_IN_BUF
  // hand in a "cleanly padded" buffer to p2s
  uint8_t * in_formatted = new uint8_t[mat.nrows_a * mat.ncols_a];
  memset(in_formatted, 0, mat.nrows_a * mat.ncols_a);
  memcpy(in_formatted, in, rhs.nrows * rhs.ncols);
  platform->copyBufferHostToAccel((void *)in_formatted, inbuf_p2s, nbytes_bitpar);
  delete [] in_formatted;
#else
  platform->copyBufferHostToAccel((void *)host_buf, inbuf_p2s, nbytes_bitpar);
#endif
  //use bismo input buff as out buff for p2s
  acc->setup_p2s(inbuf_p2s, nbytes, (void *) accel_buf, mat.nrows_a, mat.ncols_a, mat.nbits);
  uint32_t cycles = acc->p2s_exec_and_wait();
  BISMORT_DEBUG("[execMatMulLayer] Serialize rhs took " << cycles << " cycles");

#ifdef BISMORT_P2S_VERIFY_AGAINST_CPU
  // compare against sw version
  rhs.importRegular((uint8_t *) host_buf);
  uint8_t * rhs_hw_serialized = new uint8_t[nbytes];
  platform->copyBufferAccelToHost((void *)accel_buf, rhs_hw_serialized, nbytes);
  int memcmpres = memcmp(rhs_hw_serialized, mat.data, nbytes);
  if(memcmpres != 0){
  // copy buffer from host
#ifdef P2S_CLEAR_IN_BUF
  // that version cannot admit p2s errors
  assert(memcmpres==0);
#endif
  // platform->copyBufferHostToAccel(rhs.data, (void *)dsc.accel_buf_in, dsc.nbytes_buf_in);
  BISMORT_DEBUG("[execMatMulLayer] expected: ");
#ifdef DEBUG
  rhs.printHex();
#endif
  BISMORT_DEBUG("[execMatMulLayer] found: ");
  memcpy(mat.data, rhs_hw_serialized, nbytes);
#ifdef DEBUG
  rhs.printHex();
#endif
  } else {
    BISMORT_DEBUG("[execMatMulLayer] P2S hw execution fine");
  }
#endif
#endif
}


void execThresLayer(LayerHandle id, const int32_t * in, uint8_t * out) {
  // TODO implement execThresLayer
}

// destroy layer with given handle
void deinitLayer(LayerHandle id) {
  // for now this does nothing -- we rely on global init/deinit
  // TODO implement a better resource management strategy
}

}
