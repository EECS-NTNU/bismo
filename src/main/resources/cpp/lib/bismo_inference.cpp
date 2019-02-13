#include "bismo_inference_internal.hpp"

namespace bismo_inference {
// global handle for the platform and BISMO driver
WrapperRegDriver * platform;
BitSerialMatMulAccelDriver * acc;
HardwareCfg cfg;
uint32_t weightOCMBase, weightOCMBytesLeft;
uint32_t activationOCMBase, activationOCMBytesLeft;
uint32_t thresholdOCMBase, thresholdOCMBytesLeft;
std::vector<InternalLayerDescriptor> registry;

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

uint32_t allocActivationOCM(size_t nbytes) {
  // since we follow a strictly layer-by-layer strategy where only one
  // layer executes at a time, we simply use the same OCM buffer for all layers
  // check if enough space in activation OCM
  BISMORT_DEBUG("[allocActivationOCM] alloc " << nbytes << ", available " << activationOCMBytesLeft);
  assert(nbytes <= activationOCMBytesLeft);
  // all layers use the same activation buffer, so no base ptr update
  return 0;
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
  frc.bram_id_range = bram_range;
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


// parameter shape: thresholds[nthresholds][nchannels]
LayerHandle initThresLayer(ThresLayerDescriptor & dsc, const uint8_t * thresholds) {
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
