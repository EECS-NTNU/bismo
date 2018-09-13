// Copyright (c) 2018 Xilinx
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
import fpgatidbits.streams._

// Generators that take in descriptors and generate sequences of BISMO
// instructions. here for the Execute stage.
// In the case where the matrix multiply workload is predictable (e.g. known
// shape and size, repeated many times) this saves a tremendous amount of
// instruction storage and bandwidth.

// descriptor for a bit-serial matrix multiply
class MMDescriptor extends PrintableBundle {
  // number of DPA tiles in each spatial direction (for a single binary matrix)
  val tiles_m = UInt(width = BISMOLimits.inpBufAddrBits)
  val tiles_k = UInt(width = BISMOLimits.inpBufAddrBits)
  val tiles_n = UInt(width = BISMOLimits.inpBufAddrBits)
  // bitwidth of input matrices
  val bits_l = UInt(width = BISMOLimits.maxShiftBits / 2)
  val bits_r = UInt(width = BISMOLimits.maxShiftBits / 2)

  val printfStr: String = "Tm=%d Tk%d Tn=%d l=%d r=%d \n"
  val printfElems = {() => Seq[Node](tiles_m, tiles_k, tiles_n, bits_l, bits_r)}

  override def cloneType: this.type =
    new MMDescriptor().asInstanceOf[this.type]
}

// descriptor for a single bit-serial matmul including buffer base addrs
class SingleMMDescriptor extends MMDescriptor {
  // base addresses for input matrices
  val base_l = UInt(width = BISMOLimits.inpBufAddrBits)
  val base_r = UInt(width = BISMOLimits.inpBufAddrBits)

  override val printfStr: String = "Tm=%d Tk%d Tn=%d l=%d r=%d base_l=%d base_r=%d \n"
  override val printfElems = {() => Seq[Node](tiles_m, tiles_k, tiles_n, bits_l, bits_r, base_l, base_r)}
  override def cloneType: this.type =
    new SingleMMDescriptor().asInstanceOf[this.type]
}

// descriptor for several bit-serial matmuls of the same shape/size, including
// how many buffer regions are available for latency hiding
class MultiMMDescriptor extends SingleMMDescriptor {
  val num_reps = UInt(width = BISMOLimits.maxRepBits)
  val bregions_l = UInt(width = BISMOLimits.maxBufRegionBits)
  val bregions_r = UInt(width = BISMOLimits.maxBufRegionBits)

  override val printfStr: String = "Tm=%d Tk%d Tn=%d l=%d r=%d base_l=%d base_r=%d bregions_l=%d bregions_r=%d \n"
  override val printfElems = {() => Seq[Node](tiles_m, tiles_k, tiles_n, bits_l, bits_r, base_l, base_r, bregions_l, bregions_r)}
  override def cloneType: this.type =
    new MultiMMDescriptor().asInstanceOf[this.type]
}

// instruction generator for a single bit-serial matrix multiply
// each descriptor from "in" will turn into a sequence of instructions on "out"
class ExecInstrGenSingleMM extends Module {
  val io = new Bundle {
    val in = Decoupled(new SingleMMDescriptor()).flip
    val out = Decoupled(new BISMOInstruction())
  }
  // TODO
}

// instruction generator for a single bit-serial matrix multiply
// each descriptor from "in" will turn into a sequence of instructions on "out"
class ExecInstrGenMultiMM extends Module {
  val io = new Bundle {
    val in = Decoupled(new MultiMMDescriptor()).flip
    val out = Decoupled(new BISMOInstruction())
  }
  // TODO
}
