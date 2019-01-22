#include "bismo_inference.hpp"
#include "BitSerialMatMulAccelDriver.hpp"

namespace bismo_inference {
// global handle for the platform and BISMO driver
WrapperRegDriver * platform;
BitSerialMatMulAccelDriver * acc;
HardwareCfg cfg;
// TODO add layer registry that keeps track of instantiated layers
// TODO add OCM resource pool to keep track of OCM use for weights/thresholds
uint32_t weightOCMBase, weightOCMBytesLeft;
uint32_t activationOCMBase, activationOCMBytesLeft;
uint32_t thresholdOCMBase, thresholdOCMBytesLeft;

// global init/deinit for the runtime library
void init() {
  platform = initPlatform();
  acc = new BitSerialMatMulAccelDriver(platform);
  acc->reset();
  acc->print_hwcfg_summary();
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

uint32_t allocWeightOCM(size_t nbytes) {
  // check if enough weight OCM is left
  // TODO: if not enough space, fail gracefully to permit SW execution
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
  assert(nbytes <= activationOCMBytesLeft);
  // all layers use the same activation buffer, so no base ptr update
  return 0;
}

// initialize layer of given type and return handle
// parameter shape: weights[M][K]
LayerHandle initMatMulLayer(MatMulLayerDescriptor & dsc, const uint8_t * weights) {
  // allocate OCM space for weights
  gemmbitserial::GEMMContext ctx = acc->allocGEMMContext(
    dsc.M, dsc.K, dsc.N, dsc.wbits, dsc.ibits, dsc.wsigned, dsc.isigned
  );
  size_t wbytes = ctx.lhs.wordsPerBitplane() * ctx.lhs.nbits * sizeof(PackedBitGroupType);
  uint32_t wbuf_base = allocWeightOCM(wbytes);
  // convert weights to bit serial
  // don't really care about performance here since this is one-off
  ctx.lhs.importRegular(weights);
  // allocate DRAM buffer and copy bit-serial weights there
  // TODO optimization: can use a single DRAM buffer whose size is the largest
  // weight buffer since this is done only once per layer
  void * accelLHS = platform->allocAccelBuffer(wbytes);
  platform->copyBufferHostToAccel((void *)weights, accelLHS, wbytes);
  // create instruction sequence to fetch weights into OCM
  // TODO this needs to be checked for fetch width != exec width
  // (see exec_to_fetch_width_ratio in BitSerialMatMulExecutor)
  assert(cfg.dpaDimCommon == cfg.readChanWidth);
  Op fetchOp;
  FetchRunCfg fetchRunCfg;
  fetchOp.opcode = opRun;
  fetchRunCfg.bram_addr_base = wbuf_base;
  fetchRunCfg.bram_id_start = acc->get_fetch_first_lhs_id();
  fetchRunCfg.bram_id_range = cfg.dpaDimLHS - 1;
  fetchRunCfg.dram_base = accelLHS;
  fetchRunCfg.dram_block_offset_bytes = 0;
  fetchRunCfg.dram_block_size_bytes = wbytes;
  fetchRunCfg.dram_block_count = 1;
  fetchRunCfg.tiles_per_row = ctx.lhs.ncols_a / cfg.dpaDimCommon;
  acc->set_stage_enables(0, 0, 0, 0);
  acc->push_fetch_op(fetchOp);
  acc->push_fetch_runcfg(fetchRunCfg);
  assert(acc->fetch_opcount() == 1);
  // launch weight fetch and wait until complete
  acc->set_stage_enables(1, 0, 0, 0);
  while(acc->fetch_opcount() != 0);
  acc->set_stage_enables(0, 0, 0, 0);

  // TODO create instruction sequence for execution, store for later
  // TODO create entry in layer registry
  // TODO return layer handle
  return 0;
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

// parameter shape: weights[ofm][ifm][ksize][ksize]
LayerHandle initConvLayer(ConvLayerDescriptor & dsc, const uint8_t * weights) {
  // TODO allocate OCM space for weights
  // TODO convert weights to bit serial
  // TODO copy bit-serial weights into DRAM
  // TODO create and run instruction sequence to fetch weights into OCM
  // TODO create instruction sequence for execution, store for later
  // TODO create entry in layer registry
  // TODO return layer handle
  return 0;
}

// execute layer with given handle
// in and out are assumed to be preallocated to appropriate buffer sizes,
// depending on the type of layer
void execMatMulLayer(LayerHandle id, const uint8_t * in, int32_t * out) {
  // TODO implement execMatMulLayer
}

void execThresLayer(LayerHandle id, const int32_t * in, uint8_t * out) {
  // TODO implement execThresLayer
}

void execConvLayer(LayerHandle id, const uint8_t * in, int32_t * out) {
  // TODO implement execConvLayer
}

// destroy layer with given handle
void deinitLayer(LayerHandle id) {
  // for now this does nothing -- we rely on global init/deinit
  // TODO implement a better resource management strategy
}

}
