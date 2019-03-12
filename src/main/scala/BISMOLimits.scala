// Copyright (c) 2018 Xilinx
// Copyright (c) 2018 Norwegian University of Science and Technology (NTNU)
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

package bismo

import Chisel._

object BISMOLimits {
  val cnvImgSizeBits = 8
  val cnvKernelSizeBits = 4
  val cnvStrideBits = 4
  val cnvPadBits = 4
  val fetchIDBits = 9
  val fetchBurstBeats = 1
  val inpBufAddrBits = 16
  val dramAddrBits = 32
  val dramBlockSizeBits = 16
  val dramBlockCountBits = 8
  val dramBlockOffsBits = 24
  val maxShift = 16
  val maxShiftBits = log2Up(maxShift+1)
  val resAddrBits = 1
  val instrBits = 128
  val ifgBits = 32
  val maxBufRegions = 8
  val maxBufRegionBits = log2Up(maxBufRegions)
  val maxRepBits = 16
  val descrBits = 208
  val swuDescrInBits = inpBufAddrBits + cnvImgSizeBits + cnvKernelSizeBits + cnvStrideBits
  val swuDescrOutBits = inpBufAddrBits
  val numStages = 3
  val execAddrGenOutBits = 43
}
