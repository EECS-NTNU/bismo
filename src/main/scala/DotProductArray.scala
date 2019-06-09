// Copyright (c) 2018 Norwegian University of Science and Technology (NTNU)
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
// * Neither the name of BISMO nor the names of its
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
import fpgatidbits.synthutils.PrintableParam

// The DotProductArray is a two-dimensional array of DotProductUnits,
// computing/accumulating a bit-serial matrix multiplication every cycle.

class DotProductArrayParams(
  // parameters for each DotProductUnit
  val dpuParams: DotProductUnitParams,
  // dot product array dimensions
  val m: Int, // rows of left-hand-side matrix (LHS) per cycle
  val n: Int, // cols of right-hand-side matrix (RHS) per cycle
  // extra register levels in broadcast interconnect
  val extraPipelineRegs: Int = 0) extends PrintableParam {
  // latency of instantiated DPUs
  val dpuLatency: Int = dpuParams.getLatency()
  // contributed latency of DPA due to interconnect pipelining
  val myLatency: Int = 1 + extraPipelineRegs
  def getLatency(): Int = {
    return myLatency + dpuLatency
  }
  def headersAsList(): List[String] = {
    return dpuParams.headersAsList() ++ List("M", "N")
  }

  def contentAsList(): List[String] = {
    return dpuParams.contentAsList() ++ List(m, n).map(_.toString)
  }
}

class DotProductArray(val p: DotProductArrayParams) extends Module {
  val io = new Bundle {
    // inputs broadcasted to each DPU
    val valid = Bool(INPUT)
    val shiftAmount = UInt(INPUT, width = BISMOLimits.maxShiftBits)
    val negate = Bool(INPUT)
    val clear_acc = Bool(INPUT)
    // DPU bit inputs, connected appropriately to 2D array
    val a = Vec.fill(p.m) { Bits(INPUT, width = p.dpuParams.inpWidth) }
    val b = Vec.fill(p.n) { Bits(INPUT, width = p.dpuParams.inpWidth) }
    // DPU outputs from each accumulator
    val out = Vec.fill(p.m) { Vec.fill(p.n) { UInt(OUTPUT, width = p.dpuParams.accWidth) } }
  }

  // instantiate the array of DPUs
  val dpu = Vec.fill(p.m) {
    Vec.fill(p.n) {
      Module(new DotProductUnit(p.dpuParams)).io
    }
  }

  // connect the array of DPUs to the inputs
  for (i ← 0 to p.m - 1) {
    for (j ← 0 to p.n - 1) {
      // common broadcast inputs
      dpu(i)(j).in.valid := ShiftRegister(io.valid, p.myLatency)
      dpu(i)(j).in.bits.acc_shift := ShiftRegister(io.shiftAmount(0), p.myLatency)
      dpu(i)(j).in.bits.neg := ShiftRegister(io.negate, p.myLatency)
      dpu(i)(j).in.bits.clear := ShiftRegister(io.clear_acc, p.myLatency)
      // dot product bit inputs, connect along rows and columns
      dpu(i)(j).in.bits.a := ShiftRegister(io.a(i), p.myLatency)
      dpu(i)(j).in.bits.b := ShiftRegister(io.b(j), p.myLatency)
      // expose accumulators
      io.out(i)(j) := dpu(i)(j).out
    }
  }
  // NOTE: the current DPA interconnect is not systolic (i.e. no regs between
  // rows/cols), which simplifies control and increases utilization, but may
  // limit scaling to large DPA sizes.
}
