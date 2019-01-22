#include "bismo_inference.hpp"
#include "BitSerialMatMulAccelDriver.hpp"

namespace bismo_inference {
// global handle for the platform and BISMO driver
WrapperRegDriver * platform;
BitSerialMatMulAccelDriver * acc;
// TODO add layer registry that keeps track of instantiated layers
// TODO add OCM resource pool to keep track of OCM use for weights/thresholds

// global init/deinit for the runtime library
void init() {
  platform = initPlatform();
  acc = new BitSerialMatMulAccelDriver(platform);
  acc->print_hwcfg_summary();
  // TODO initialize OCM resource pool from hwcfg
}

void deinit() {
  delete acc;
  deinitPlatform(platform);
}

uint32_t allocWeightOCM(size_t nbytes) {
  // TODO check if enough weight OCM is left
  // TODO increment pointer to next available OCM slot
  // TOOD return allocated base address
  return 0;
}

uint32_t allocThresOCM(size_t nbytes) {
  // TODO check if enough weight OCM is left
  // TODO increment pointer to next available OCM slot
  // TOOD return allocated base address
  return 0;
}

uint32_t allocActivationOCM(size_t nbytes) {
  // since we follow a strictly layer-by-layer strategy where only one
  // layer executes at a time, we simply use the same OCM buffer for all layers
  // TODO check if enough space in activation OCM
  return 0;
}

// initialize layer of given type and return handle
// parameter shape: weights[M][K]
LayerHandle initMatMulLayer(MatMulLayerDescriptor & dsc, const uint8_t * weights) {
  // TODO allocate OCM space for weights
  // TODO convert weights to bit serial
  // TODO copy bit-serial weights into DRAM
  // TODO create and run instruction sequence to fetch weights into OCM
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
