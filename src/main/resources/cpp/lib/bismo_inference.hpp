// Copyright (c) 2019 Xilinx
//
// BSD v3 License
//
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// * Redistributions of source code must retain the above copyright notice, this
//   list of conditions and the following disclaimer.
//
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
//
// * Neither the name of [project] nor the names of its
//   contributors may be used to endorse or promote products derived from
//   this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#include <stdint.h>

namespace bismo_inference {
// handle representing a particular layer instance that BISMO knows
// how to execute
typedef unsigned int LayerHandle;

// struct with details of currently instantiated hardware config
// copied from BitSerialMatMulAccelDriver in order not to have that as a
// dependency
typedef struct {
  uint64_t accWidth;
  uint64_t cmdQueueEntries;
  uint64_t dpaDimCommon;
  uint64_t dpaDimLHS;
  uint64_t dpaDimRHS;
  uint64_t lhsEntriesPerMem;
  uint64_t maxShiftSteps;
  uint64_t readChanWidth;
  uint64_t rhsEntriesPerMem;
  uint64_t writeChanWidth;
} HardwareConfig;

// global init/deinit for the runtime library
void init();
void deinit();

// benchmark host<->accel transfer times
void benchmark_host_accel_transfer();

// retrieve hardware configuration for the instance
HardwareConfig getHardwareConfig();

// descriptor structs that contain layer properties for each supported
// layer type

// properties for matrix multiply (inner product) layers
typedef struct {
  uint8_t wbits;  // bits per weight
  uint8_t ibits;  // bits per input
  bool wsigned;   // whether weights are signed
  bool isigned;   // whether inputs are signed
  uint32_t M;     // rows of left-hand-side (weight) matrix
  uint32_t K;     // common dimension (columns)
  uint32_t N;     // rows of right-hand-side (weight) matrix
  // note that the right-hand-side matrix is assumed transposed
} MatMulLayerDescriptor;

// properties for thresholding layers
typedef struct {
  uint32_t nchannels;   // number of channels
  uint8_t nthresholds;  // number of thresholds per channel
  uint16_t idim;        // input image dimension (assumed width=height)
} ThresLayerDescriptor;

// properties for convolutional layers
typedef struct {
  uint8_t wbits;  // bits per weight
  uint8_t ibits;  // bits per input
  bool wsigned;   // whether weights are signed
  bool isigned;   // whether inputs are signed
  uint8_t pad;    // zero padding on each side of input image
  uint8_t ksize;  // convolution kernel size (asssumed width=height)
  uint8_t stride; // convolution kernel stride (assumed horizontal=vertical)
  uint16_t idim;  // input image dimension (assumed width=height)
  uint16_t ifm;   // number of input channels
  uint16_t ofm;   // number of output channels
  bool useCPULowering;  // whether to use CPU (software) matmul lowering
} ConvLayerDescriptor;

// initialize layer of given type and return handle
// parameter shape: weights[M][K]
LayerHandle initMatMulLayer(MatMulLayerDescriptor & dsc, const uint8_t * weights, bool cpu_only = false);
// parameter shape: thresholds[nthresholds][nchannels]
LayerHandle initThresLayer(ThresLayerDescriptor & dsc, const uint8_t * thresholds, bool cpu_only = false);
// parameter shape: weights[ofm][ifm][ksize][ksize]
LayerHandle initConvLayer(ConvLayerDescriptor & dsc, const uint8_t * weights, bool cpu_only = false);

// execute layer with given handle
// in and out are assumed to be preallocated to appropriate buffer sizes,
// depending on the type of layer
void execMatMulLayer(LayerHandle id, const uint8_t * in, int32_t * out);
void execThresLayer(LayerHandle id, const int32_t * in, uint8_t * out);
void execConvLayer(LayerHandle id, const uint8_t * in, int32_t * out);

// destroy layer with given handle
void deinitLayer(LayerHandle id);
}
