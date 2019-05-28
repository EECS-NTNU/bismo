#include "bismo_inference_internal.hpp"

namespace bismo_inference {
TIMER_INIT();
// global handle for the platform and BISMO driver
WrapperRegDriver * platform;
BitSerialMatMulAccelDriver * acc;
HardwareCfg cfg;
//std::vector<InternalLayerDescriptor> registry;
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
  // allocate shared buffer for p2s
  accel_p2s_bitpar_buffer = (uint32_t)(uint64_t) platform->allocAccelBuffer(BISMORT_P2S_BITPAR_BYTES);
  host_p2s_bitpar_buffer = new uint8_t[BISMORT_P2S_BITPAR_BYTES];
}

void deinit() {
  delete acc;
  delete [] host_p2s_bitpar_buffer;
  platform->deallocAccelBuffer((void *) accel_p2s_bitpar_buffer);
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
    TIMER_REPORT("host2accel");
    TIMER_SAMPLE();
    platform->copyBufferAccelToHost(accelbuf, hostbuf, nbytes);
    TIMER_SAMPLE();
    TIMER_REPORT("accel2host");
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
/*
void genFetchInstrs(
  std::vector<BISMOInstruction> & ins,
  size_t bram_base,
  bool lhsNotRhs,
  uint32_t dram_base,
  size_t tiles_per_row,
  size_t nbytes
) {
  BISMOFetchRunInstruction frc, frc_ratio_fixed;
  size_t bram_start = lhsNotRhs ? acc->get_fetch_first_lhs_id() : acc->get_fetch_first_rhs_id();
  size_t bram_range = (lhsNotRhs ? cfg.dpaDimLHS : cfg.dpaDimRHS) - 1;
  size_t exec_to_fetch_width_ratio = cfg.dpaDimCommon / cfg.readChanWidth;

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
    frc_ratio_fixed = frc;
    frc_ratio_fixed.tiles_per_row *= exec_to_fetch_width_ratio;
    frc_ratio_fixed.bram_addr_base *= exec_to_fetch_width_ratio;
    ins.push_back(frc_ratio_fixed.asRaw());
    BISMORT_DEBUG("[genFetchInstrs] " << frc);
    size_t last_chunk_bytes = frc.dram_block_count * frc.dram_block_size_bytes;
    bytes_left -= last_chunk_bytes;
    frc.dram_base += last_chunk_bytes;
    frc.bram_addr_base += last_chunk_bytes / bytes_per_addr;
  }
}*/

/*
// parameter shape: thresholds[nthresholds][nchannels]
LayerHandle initThresLayer(ThresLayerDescriptor & dsc, const uint8_t * thresholds, bool cpu_only) {
  // TODO allocate OCM space for thresholds
  // TODO write thresholds into OCM
  // TODO create instruction sequence for execution, store for later
  // TODO create entry in layer registry
  // TODO return layer handle
  return 0;
}

void execThresLayer(LayerHandle id, const int32_t * in, uint8_t * out) {
  // TODO implement execThresLayer
}
*/
// destroy layer with given handle
/*
void deinitLayer(LayerHandle id) {
  InternalLayerDescriptor dsc = registry[id];
  switch (dsc.layerType) {
    case layerMatMul:
      // dealloc associated sw gemmbitserial context
      gemmbitserial::deallocGEMMContext(dsc.ctx);
      if(!dsc.cpu_only) {
        platform->deallocAccelBuffer((void *) dsc.accel_buf_in_lhs);
        platform->deallocAccelBuffer((void *) dsc.accel_buf_in_rhs);
        platform->deallocAccelBuffer((void *) dsc.accel_buf_out);
        delete [] dsc.padded_result_host_buffer;
      }
      // TODO free up associated weight OCM
      break;
    case layerConv:
      // deinit sw impl
      gemmbitserial::deallocConvBitSerialContext(dsc.cnv_ctx);
      delete [] dsc.cnv_lowering_buf;
      delete [] dsc.transpose_result_host_buffer;
      // deinit associated matmul layer
      deinitLayer(dsc.cnv_matmul_handle);
      break;
    case layerThres:
    // TODO deallocate once we have threshold layers accelerated
      break;
    default:
      throw "Unrecognized layer type in deinitLayer";
  }
  // TODO mark descriptor as invalid or remove from registry
}*/

}
