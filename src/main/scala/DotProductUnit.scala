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

// The DotProductUnit computes a binary dot product over time,
// with possibilities for shifting (for weighting by powers-of-two)
// and negative contributions (for signed numbers).
// structurally, it is a AND-popcount-shift-accumulate datapath.

class DotProductUnitParams(
  // popcount module input width (bits per cycle)
  val pcParams: PopCountUnitParams,
  // width of accumulator register
  val accWidth: Int,
  // maximum number of shift steps
  val maxShiftSteps: Int,
  // do not instantiate the shift stage
  val noShifter: Boolean = false,
  // do not instantiate the negate stage
  val noNegate: Boolean = false,
  // extra pipeline regs for retiming
  val extraPipelineRegs: Int = 0,
  // use an optimized VHDL compressor generator
  val useVhdlCompressor: Boolean = true,
  // number of regs for the VHDL compressor (if used)
  // TODO set this to -1 once supported to use default value
  val vhdlCompressorRegs: Int = 0
) extends PrintableParam {
  // parameters for BlackBoxCompressor (if any)
  val bbCompParams = new BlackBoxCompressorParams(
    N = pcParams.numInputBits, D = vhdlCompressorRegs
  )
  // internal pipeline registers inside DPU
  val myLatency = if(useVhdlCompressor){
    5 + bbCompParams.getLatency() + extraPipelineRegs
  } else {
    6 + extraPipelineRegs
  }
  // latency of instantiated PopCountUnit
  val popcountLatency: Int = if(useVhdlCompressor){0} else {pcParams.getLatency()}
  // return total latency
  def getLatency(): Int = {
    if(useVhdlCompressor) {
      return myLatency
    } else {
      return myLatency + popcountLatency
    }
  }
  def headersAsList(): List[String] = {
    return pcParams.headersAsList() ++ List("AccWidth", "NoShift", "NoNeg", "DPULatency")
  }

  def contentAsList(): List[String] = {
    return pcParams.contentAsList() ++ List(accWidth, noShifter, noNegate, getLatency()).map(_.toString)
  }
}

class DotProductStage0(p: DotProductUnitParams) extends Bundle {
  // bit vectors for dot product
  val a = Bits(width = p.pcParams.numInputBits)
  val b = Bits(width = p.pcParams.numInputBits)
  // number of steps to left shift result by before accumulation
  val shiftAmount = UInt(width = log2Up(p.maxShiftSteps+1))
  // whether to negate result before accumulation
  val negate = Bool()
  // whether to clear the accumulator before adding the new result
  val clear_acc = Bool()

  override def cloneType: this.type =
    new DotProductStage0(p).asInstanceOf[this.type]
}

// Bundles of partially-processed input through the pipelined datapath
class DotProductStage1(p: DotProductUnitParams) extends Bundle {
  // result of AND of inputs
  val andResult = Bits(width = p.pcParams.numInputBits)
  // number of steps to left shift result by before accumulation
  val shiftAmount = UInt(width = log2Up(p.maxShiftSteps))
  // whether to negate result before accumulation
  val negate = Bool()
  // whether to clear the accumulator before adding the new result
  val clear_acc = Bool()

  override def cloneType: this.type =
    new DotProductStage1(p).asInstanceOf[this.type]
}

class DotProductStage2(p: DotProductUnitParams) extends Bundle {
  // result of popcount
  val popcountResult = UInt(width = log2Up(p.pcParams.numInputBits+1))
  // number of steps to left shift result by before accumulation
  val shiftAmount = UInt(width = log2Up(p.maxShiftSteps))
  // whether to negate result before accumulation
  val negate = Bool()
  // whether to clear the accumulator before adding the new result
  val clear_acc = Bool()

  override def cloneType: this.type =
    new DotProductStage2(p).asInstanceOf[this.type]
}

class DotProductStage3(p: DotProductUnitParams) extends Bundle {
  // result of shift
  val shiftResult = UInt(width = p.accWidth)
  // whether to negate result before accumulation
  val negate = Bool()
  // whether to clear the accumulator before adding the new result
  val clear_acc = Bool()

  override def cloneType: this.type =
    new DotProductStage3(p).asInstanceOf[this.type]
}

class DotProductStage4(p: DotProductUnitParams) extends Bundle {
  // result of negate
  val negateResult = UInt(width = p.accWidth)
  // whether to clear the accumulator before adding the new result
  val clear_acc = Bool()

  override def cloneType: this.type =
    new DotProductStage4(p).asInstanceOf[this.type]
}

class DotProductUnit(val p: DotProductUnitParams) extends Module {
  val io = new Bundle {
    val in = Valid(new DotProductStage0(p)).asInput
    val out = UInt(OUTPUT, width = p.accWidth)
  }
  // extra pipeline regs at the input for retiming
  val regInput = ShiftRegister(io.in, p.extraPipelineRegs)

  // pipeline stage 0: register the input
  val regStage0_v = Reg(init = Bool(false), next = regInput.valid)
  val regStage0_b = Reg(next = regInput.bits)
  //when(regStage0_v) { printf("Stage0: a %x b %x shift %d neg %d clear %d\n", regStage0_b.a, regStage0_b.b, regStage0_b.shiftAmount, regStage0_b.negate, regStage0_b.clear_acc)}

  //Chose between a VHDL implementation for the AND-Popcount, otherwise the Chisel one
  val stage2 = (new DotProductStage2(p)).asDirectionless
  // intermediate values to abstract the level: if using vhdl based popcount no latency, otherwise variable latency
  val intermediate_valid = Bool()
  val stage2_pc_v = ShiftRegister(intermediate_valid, 0)

  if (p.useVhdlCompressor) {
    val compressor = Module(new BlackBoxCompressor(p.bbCompParams))
    val compLatency = p.bbCompParams.getLatency()
    compressor.io.c := regStage0_b.a
    compressor.io.d := regStage0_b.b
    stage2.popcountResult :=  compressor.io.r
    // need extra delays on pass-through parts due to pipelined compressor
    stage2.shiftAmount := ShiftRegister(regStage0_b.shiftAmount, compLatency)
    stage2.negate := ShiftRegister(regStage0_b.negate, compLatency)
    stage2.clear_acc := ShiftRegister(regStage0_b.clear_acc, compLatency)
    intermediate_valid := ShiftRegister(regStage0_v, compLatency)
  } else{
    // instantiate the popcount unit
    val modPopCount = Module(new PopCountUnit(p.pcParams))
    //when(io.in.valid) { printf("DPU operands are %x and %x\n", io.in.bits.a, io.in.bits.b) }
    // core AND-popcount-shift part of datapath
    // note that the valid bit and the actual pipeline contents are
    // treated differently to save FPGA resources: valid pipeline regs
    // are initialized to 0, whereas actual data regs aren't initialized
    // pipeline stage 1: AND the bit vector inputs
    val stage1 = (new DotProductStage1(p)).asDirectionless
    stage1.andResult := regStage0_b.a & regStage0_b.b
    stage1.shiftAmount := regStage0_b.shiftAmount
    stage1.negate := regStage0_b.negate
    stage1.clear_acc := regStage0_b.clear_acc
    val regStage1_v = Reg(init = Bool(false), next = regStage0_v)
    val regStage1_b = Reg(next = stage1)
    //when(regStage1_v) { printf("Stage1: andResult %x shift %d neg %d clear %d\n", regStage1_b.andResult, regStage1_b.shiftAmount, regStage1_b.negate, regStage1_b.clear_acc)}

    // pipeline stage 2: popcount the result of AND
    //val stage2 = (new DotProductStage2(p)).asDirectionless
    modPopCount.io.in := regStage1_b.andResult
    stage2.popcountResult := modPopCount.io.out
    // need extra delays on pass-through parts due to pipelined popcount
    stage2.shiftAmount := ShiftRegister(regStage1_b.shiftAmount, p.popcountLatency)
    stage2.negate := ShiftRegister(regStage1_b.negate, p.popcountLatency)
    stage2.clear_acc := ShiftRegister(regStage1_b.clear_acc, p.popcountLatency)
    val stage2_intermdiate_pc_v = ShiftRegister(regStage1_v, p.popcountLatency)
    intermediate_valid := stage2_intermdiate_pc_v

  }
  val regStage2_v = Reg(init = Bool(false), next = stage2_pc_v)
  val regStage2_b = Reg(next = stage2)
  //when(regStage2_v) { printf("Stage2: popCResult %d shift %d neg %d clear %d\n", regStage2_b.popcountResult, regStage2_b.shiftAmount, regStage2_b.negate, regStage2_b.clear_acc)}

  // pipeline stage 3: shift
  val stage3 = (new DotProductStage3(p)).asDirectionless
  if(p.noShifter) {
    stage3.shiftResult := regStage2_b.popcountResult
  } else {
    stage3.shiftResult := regStage2_b.popcountResult << regStage2_b.shiftAmount
  }
  stage3.negate := regStage2_b.negate
  stage3.clear_acc := regStage2_b.clear_acc
  val regStage3_v = Reg(init = Bool(false), next = regStage2_v)
  val regStage3_b = Reg(next = stage3)
  //when(regStage3_v) { printf("Stage3: shiftRes %d neg %d clear %d\n", regStage3_b.shiftResult, regStage3_b.negate, regStage3_b.clear_acc)}

  // pipeline stage 4: negate
  val stage4 = (new DotProductStage4(p)).asDirectionless
  val shiftRes = regStage3_b.shiftResult
  if(p.noNegate) {
    stage4.negateResult := shiftRes
  } else {
    stage4.negateResult := Mux(regStage3_b.negate, -shiftRes, shiftRes)
  }
  stage4.clear_acc := regStage3_b.clear_acc
  val regStage4_v = Reg(init = Bool(false), next = regStage3_v)
  val regStage4_b = Reg(next = stage4)
  // accumulator register for the dot product. cleared with clear_acc
  val regAcc = Reg(outType = UInt(width = p.accWidth))
  //when(regStage4_v) { printf("Stage4: negResult %d clear %d acc: %d\n", regStage4_b.negateResult, regStage4_b.clear_acc, regAcc)}
  // accumulate new input when valid
  when(regStage4_v) {
    when(regStage4_b.clear_acc) {
      regAcc := regStage4_b.negateResult
    } .otherwise {
      regAcc := regAcc + regStage4_b.negateResult
    }
  }

  // expose the accumulator output directly
  io.out := regAcc
}
