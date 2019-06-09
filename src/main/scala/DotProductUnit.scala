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

class DotProductUnitParams(
  // width of inputs
  val inpWidth: Int,
  // width of accumulator register
  val accWidth: Int,
  // number of regs for the VHDL compressor
  // -1 gives maximum pipelining (= compressor tree depth)
  val vhdlCompressorRegs: Int = -1,
  // whether to add an input register at the DPU input
  val doNotRegisterInput: Boolean = false,
  // use an optimized VHDL compressor generator
  val useVhdlCompressor: Boolean = true
) extends PrintableParam {
  // parameters for BlackBoxCompressor (if any)
  val bbCompParams = new BlackBoxCompressorParams(
    N = inpWidth, D = vhdlCompressorRegs
  )
  // internal pipeline registers inside DPU
  val internalPipelineRegs = if(doNotRegisterInput) {2} else {3}
  val myLatency = internalPipelineRegs + bbCompParams.getLatency()

  // return total latency
  def getLatency(): Int = {
    return myLatency
  }

  def headersAsList(): List[String] = {
    return List("InpWidth", "AccWidth", "Latency")
  }

  def contentAsList(): List[String] = {
    return List(inpWidth, accWidth, getLatency()).map(_.toString)
  }
}

class NewDotProductStage0(p: DotProductUnitParams) extends Bundle {
  // bit vectors for dot product
  val a = Bits(width = p.inpWidth)
  val b = Bits(width = p.inpWidth)
  // accumulation mode
  val acc_shift = Bool()
  // negate contribution before accumulating
  val neg = Bool()
  // zero accumulator before adding
  val clear = Bool()

  override def cloneType: this.type =
    new NewDotProductStage0(p).asInstanceOf[this.type]
}

class NewDotProductStage1(p: DotProductUnitParams) extends Bundle {
  // result of AND of inputs
  val popcountResult = UInt(width = log2Up(p.inpWidth+1))
  // accumulation mode
  val acc_shift = Bool()
  // negate contribution before accumulating
  val neg = Bool()
  // zero accumulator before adding
  val clear = Bool()

  override def cloneType: this.type =
    new NewDotProductStage1(p).asInstanceOf[this.type]
}

class DotProductUnit(val p: DotProductUnitParams) extends Module {
  val io = new Bundle {
    val in = Valid(new NewDotProductStage0(p)).asInput
    val out = UInt(OUTPUT, width = p.accWidth)
  }
  // instantiate the compressor
  val compressor = Module(
    if(p.useVhdlCompressor) {new BlackBoxCompressor(p.bbCompParams)}
    else {new BlackBoxCompressorModel(p.bbCompParams)}
  )
  val compLatency = p.bbCompParams.getLatency()
  //when(io.in.valid) { printf("Input: a %x b %x shift %d neg %d clear %d\n", io.in.bits.a, io.in.bits.b, io.in.bits.acc_shift, io.in.bits.neg, io.in.bits.clear)}
  // pipeline stage 1: compressor
  val stage1_b = (new NewDotProductStage1(p)).asDirectionless
  val stage1_v = Bool()
  stage1_b.popcountResult := compressor.io.r
  if(p.doNotRegisterInput) {
    // give input directly to compressor
    compressor.io.c := io.in.bits.a
    compressor.io.d := io.in.bits.b
    stage1_b.acc_shift := ShiftRegister(io.in.bits.acc_shift, compLatency)
    stage1_b.neg := ShiftRegister(io.in.bits.neg, compLatency)
    stage1_b.clear := ShiftRegister(io.in.bits.clear, compLatency)
    stage1_v := ShiftRegister(io.in.valid, compLatency)
  } else {
    // pipeline stage 0: register the input
    val regStage0_v = Reg(init = Bool(false), next = io.in.valid)
    val regStage0_b = Reg(next = io.in.bits)
    compressor.io.c := regStage0_b.a
    compressor.io.d := regStage0_b.b
    stage1_b.acc_shift := ShiftRegister(regStage0_b.acc_shift, compLatency)
    stage1_b.neg := ShiftRegister(regStage0_b.neg, compLatency)
    stage1_b.clear := ShiftRegister(regStage0_b.clear, compLatency)
    stage1_v := ShiftRegister(regStage0_v, compLatency)
  }
  val regStage1_b = Reg(next = stage1_b)
  val regStage1_v = Reg(next = stage1_v)
  // pipeline stage 2: accumulate according to mode
  val regAcc = Reg(outType = SInt(width = p.accWidth))
  // accumulate new input when valid
  when(regStage1_v) {
    val acc_modes = Vec(Seq[SInt](regAcc, regAcc << 1))
    val acc = acc_modes(regStage1_b.acc_shift)
    val contr = regStage1_b.popcountResult.zext()
    regAcc := Mux(regStage1_b.clear, SInt(0, width = p.accWidth), acc) + Mux(regStage1_b.neg, -contr, contr)
    //printf("Accumulate: regAcc = %d shiftAcc? %d clear? %d contr %d \n", regAcc, regStage1_b.acc_shift, regStage1_b.clear, contr)
  }
  // expose the accumulator output directly
  io.out := regAcc
}
