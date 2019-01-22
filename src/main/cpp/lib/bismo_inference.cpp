#include "bismo_inference.hpp"
#include "BitSerialMatMulAccelDriver.hpp"

namespace bismo_inference {
// global init/deinit for the runtime library
void init() {
  // TODO implement global init
  WrapperRegDriver * platform = initPlatform();
  BitSerialMatMulAccelDriver * acc = new BitSerialMatMulAccelDriver(platform);
  acc->print_hwcfg_summary();
  delete acc;
  deinitPlatform(platform);
}

void deinit() {
  // TODO implement global deinit
}

// initialize layer of given type and return handle
// parameter shape: weights[M][K]
LayerHandle initMatMulLayer(MatMulLayerDescriptor & dsc, const uint8_t * weights) {
  // TODO implement initMatMulLayer
  return 0;
}

// parameter shape: thresholds[nthresholds][nchannels]
LayerHandle initThresLayer(ThresLayerDescriptor & dsc, const uint8_t * thresholds) {
  // TODO implement initThresLayer
  return 0;
}

// parameter shape: weights[ofm][ifm][ksize][ksize]
LayerHandle initConvLayer(ConvLayerDescriptor & dsc, const uint8_t * weights) {
  // TODO implement initConvLayer
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
  // TODO implement deinitLayer
}

}
