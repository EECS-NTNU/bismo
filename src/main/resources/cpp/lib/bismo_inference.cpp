#include "bismo_inference.hpp"
#include "BitSerialMatMulAccelDriver.hpp"
#include <vector>
#include <string.h>

#ifdef DEBUG
#define BISMORT_DEBUG(x) cout << x << endl;
#else
#define BISMORT_DEBUG(x) ;
#endif

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

typedef enum {layerMatMul, layerConv, layerThres} InternalLayerType;

typedef struct {
  InternalLayerType layerType;
  uint32_t accel_buf_in;
  uint32_t accel_buf_out;
  gemmbitserial::GEMMContext ctx;
  size_t nbytes_buf_in;
  size_t nbytes_buf_out;
  MatMulLayerDescriptor mm_dsc;
  ConvLayerDescriptor cnv_dsc;
  ThresLayerDescriptor thr_dsc;
  size_t wbase;
  size_t abase;
} InternalLayerDescriptor;

std::vector<InternalLayerDescriptor> registry;
typedef int32_t AccumType;

// global init/deinit for the runtime library
void init() {
  platform = initPlatform();
  acc = new BitSerialMatMulAccelDriver(platform);
  acc->reset();
  // currently the runtime is implemented with direct instruction feed
  // will switch to descriptors when the correct generators are impl'd
  acc->init_resource_pools();
  acc->useDirectInstructionFeed();
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
  size_t abytes = ctx.rhs.wordsPerBitplane() * ctx.rhs.nbits * sizeof(PackedBitGroupType);
  size_t resbytes = ctx.lhs.nrows_a * ctx.rhs.nrows_a * sizeof(AccumType);
  uint32_t wbase = allocWeightOCM(wbytes);
  uint32_t abase = allocActivationOCM(abytes);
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
  BISMOFetchRunInstruction frc;
  frc.targetStage = stgFetch;
  frc.isRunCfg = 1;
  frc.bram_addr_base = wbase;
  frc.bram_id_start = acc->get_fetch_first_lhs_id();
  frc.bram_id_range = cfg.dpaDimLHS - 1;
  frc.dram_base = accel_lhs_ptr;
  frc.dram_block_offset_bytes = 0;
  // adjust blocking to respect maximum fetch block size
  if(wbytes > FETCH_BLOCK_MAX) {
    frc.dram_block_size_bytes = FETCH_BLOCK_MAX;
    frc.dram_block_count = wbytes / FETCH_BLOCK_MAX;
  } else {
    frc.dram_block_size_bytes = wbytes;
    frc.dram_block_count = 1;
  }
  frc.tiles_per_row = ctx.lhs.ncols_a / cfg.dpaDimCommon;
  acc->set_stage_enables(0, 0, 0);
  acc->pushInstruction(frc.asRaw());
  assert(acc->fetch_opcount() == 1);
  BISMORT_DEBUG("[initMatMulLayer] created weight init instruction");
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
  idsc.accel_buf_in = (uint32_t)(uint64_t)platform->allocAccelBuffer(abytes);
  BISMORT_DEBUG("[initMatMulLayer] accel_buf_in: " << (idsc.accel_buf_in) << " for " << abytes << " bytes");
  // TODO optimization: can use common buffer for all results, since 1 layer
  // at a time
  idsc.accel_buf_out = (uint32_t)(uint64_t)platform->allocAccelBuffer(resbytes);
  BISMORT_DEBUG("[initMatMulLayer] accel_buf_out: " << (idsc.accel_buf_out) << " for " << resbytes << " bytes");
  LayerHandle ret = registry.size();
  BISMORT_DEBUG("[initMatMulLayer] registered new matmul layer with id " << ret);
  registry.push_back(idsc);
  // instruction generation for the rest of the execution is done dynamically
  // in the execLayer calls for now
  return ret;
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
  // TODO this needs to be checked for fetch width != exec width
  // (see exec_to_fetch_width_ratio in BitSerialMatMulExecutor)
  assert(cfg.dpaDimCommon == cfg.readChanWidth);
  const size_t k_tiles = lhs.ncols_a / cfg.dpaDimCommon;

  // TODO call p2s here instead
  rhs.importRegular((uint8_t *)in);
  // copy buffer from host
  platform->copyBufferHostToAccel(rhs.data, (void *)dsc.accel_buf_in, dsc.nbytes_buf_in);
  // enable all stages
  acc->set_stage_enables(1, 1, 1);
  // fetch the rhs matrix into the on-chip buffer
  BISMOSyncInstruction sync;
  sync.isRunCfg = 0;
  BISMOFetchRunInstruction frc;
  frc.isRunCfg = 1;
  frc.targetStage = stgFetch;
  BISMOExecRunInstruction erc;
  erc.isRunCfg = 1;
  erc.targetStage = stgExec;
  erc.writeAddr = 0;
  BISMOResultRunInstruction rrc;
  rrc.isRunCfg = 1;
  rrc.targetStage = stgResult;
  // fetch receive token from exec stage for buffer access
  // in the current schedule this token represents the entire RHS buffer
  sync.targetStage = stgFetch;
  sync.isSendToken = 0;
  sync.chanID = 0;
  acc->pushInstruction(sync.asRaw());
  frc.bram_addr_base = abase;
  frc.bram_id_start = acc->get_fetch_first_rhs_id();
  frc.bram_id_range = cfg.dpaDimRHS - 1;
  frc.dram_base = (uint32_t)(uint64_t)dsc.accel_buf_in;
  // adjust blocking to respect maximum fetch block size
  if(dsc.nbytes_buf_in > FETCH_BLOCK_MAX) {
    frc.dram_block_size_bytes = FETCH_BLOCK_MAX;
    frc.dram_block_count = dsc.nbytes_buf_in / FETCH_BLOCK_MAX;
  } else {
    frc.dram_block_size_bytes = dsc.nbytes_buf_in;
    frc.dram_block_count = 1;
  }
  frc.dram_block_offset_bytes = 0;
  frc.tiles_per_row = lhs.ncols_a / cfg.dpaDimCommon;
  BISMORT_DEBUG("[execMatMulLayer] frc for rhs fetch = " << frc);
  acc->pushInstruction(frc.asRaw());
  // fetch sends token to execute stage
  sync.targetStage = stgFetch;
  sync.isSendToken = 1;
  sync.chanID = 0;
  acc->pushInstruction(sync.asRaw());
  //acc->set_stage_enables(0, 0, 0);
  // exec receives token from fetch stage
  sync.targetStage = stgExec;
  sync.isSendToken = 0;
  sync.chanID = 0;
  acc->pushInstruction(sync.asRaw());

  // TODO this assumes activations fit into OCM, need extra tiling otherwise
  // TODO optimization: do this in smaller chunks for fetch-exec concurrency
  for(size_t lhs_tile = 0; lhs_tile < lhs_tiles; lhs_tile++) {
    for(size_t rhs_tile = 0; rhs_tile < rhs_tiles; rhs_tile++) {
      // exec stage ==============================================
      // exec receives token from result stage (acquire empty res buffer)
      sync.targetStage = stgExec;
      sync.isSendToken = 0;
      sync.chanID = 1;
      acc->pushInstruction(sync.asRaw());
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
          // TODO more flexible multi-buffering here
          erc.writeAddr = (erc.writeAddr == 0) ? 1 : 0;
          acc->pushInstruction(erc.asRaw());
        }
      }
      // exec sends token to result stage (send full res buffer)
      sync.targetStage = stgExec;
      sync.isSendToken = 1;
      sync.chanID = 1;
      acc->pushInstruction(sync.asRaw());
      // result stage =============================================
      // res receives token from exec stage (acquire full res buffer)
      sync.targetStage = stgResult;
      sync.isSendToken = 0;
      sync.chanID = 0;
      acc->pushInstruction(sync.asRaw());
      // generate write instruction
      uint32_t lhs_ind = cfg.dpaDimLHS * lhs_tile;
      uint32_t rhs_ind = cfg.dpaDimRHS * rhs_tile;
      size_t ind = rhs_ind * cfg.dpaDimLHS + lhs_ind;
      rrc.dram_base = (rptr + (ind * sizeof(AccumType)));
      rrc.dram_skip = cfg.dpaDimLHS * sizeof(AccumType);
      rrc.waitCompleteBytes = 0;
      acc->pushInstruction(rrc.asRaw());
      // res sends token to exec stage (send empty res buffer)
      sync.targetStage = stgResult;
      sync.isSendToken = 1;
      sync.chanID = 0;
      acc->pushInstruction(sync.asRaw());
    }
  }
  // exec sends token to fetch stage
  sync.targetStage = stgExec;
  sync.isSendToken = 1;
  sync.chanID = 0;
  acc->pushInstruction(sync.asRaw());

  // wait until all writes are completed
  while(acc->get_completed_writes() != dsc.nbytes_buf_out) {
    BISMORT_DEBUG("[execMatMulLayer] waiting for exec, ops f/e/r: " << acc->fetch_opcount() << " " << acc->exec_opcount() << " " << acc->res_opcount());
  };
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
    for(int i = 0; i < dsc.nbytes_buf_out / sizeof(AccumType); i++) {
      if(dsc.ctx.res[i] != out[i]) {
        cout << "pos " << i << ": " << dsc.ctx.res[i] << " " << out[i] << endl;
      }
    }
  }
#endif
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
