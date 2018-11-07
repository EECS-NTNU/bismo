// Author:  Davide Conficconi
// Date: 10/09/2018
// Revision: 0
package bismo

import Chisel._
import fpgatidbits.ocm._
import fpgatidbits.streams._

class ThrStageParams(
    // building block params
    val thuParams: ThresholdingUnitParams,
    // threshold memory depth (how many entries, address space)
    val thresholdMemDepth: Int,
    val inputMemAddr: Int,
    val resMemAddr: Int) extends PrintableParam {

  //how many threshold
  val thresholdNumber: Int = scala.math.pow(2, thuParams.maxOutputBitPrecision).toInt - 1
  // threshold memory width (how many output bits)

  val thresholdMemWidth: Int = thuParams.inputBitPrecision * thresholdNumber

  val thresholdLatency: Int = thresholdNumber - thuParams.unrollingFactorOutputPrecision

  //M of DPA
  def getUnrollRows(): Int = {
    return thuParams.unrollingFactorRows
  }
  def getRows(): Int = {
    return thuParams.matrixRows
  }
  //K of DPA
  def getInBits(): Int = {
    return thuParams.inputBitPrecision
  }
  //ASSUMPTION: No rolling in the columns
  def getCols(): Int = {
    return thuParams.matrixColumns
  }
  //ASSUMPTION: No rolling in the columns
  def getUnrollCols(): Int = {
    return thuParams.unrollingFactorColumns
  }

  def getResBitWidth(): Int = {
    return thuParams.maxOutputBitPrecision
  }
  def headersAsList(): List[String] = {
    return thuParams.headersAsList() ++ List("thresholdMemDepth", "thresholdMemWidth")
  }

  def contentAsList(): List[String] = {
    return thuParams.contentAsList() ++ List(thresholdMemDepth, thresholdMemWidth).map(_.toString)
  }
}

// interface to hardware config available to software
class ThrStageCfgIO() extends Bundle {
  val config_thu_th_unroll = UInt(OUTPUT, width = 32)
  //TODO
  override def cloneType: this.type =
    new ThrStageCfgIO().asInstanceOf[this.type]
}

// interface towards controller for the execute stage
class ThrStageCtrlIO(myP: ThrStageParams) extends PrintableBundle {

  //TODO
  // write to result memory at the end of current execution
  val writeEn = Bool()
  // result memory address to use for writing
  val writeAddr = UInt(width = log2Up(myP.thresholdMemDepth))

  override def cloneType: this.type =
    new ThrStageCtrlIO(myP).asInstanceOf[this.type]

  val printfStr = "(offs lhs/rhs = %d/%d, ntiles = %d, << %d, w? %d/%d)\n"
  val printfElems = { () ⇒
    Seq( //lhsOffset, rhsOffset, numTiles, shiftAmount, writeEn, writeAddr
    )
  }
}

// interface towards tile memories (LHS(Thresholds)/RHS(Activation) BRAMs)
class ThrTileMemIO(myP: ThrStageParams) extends Bundle {
  val thr_req = Vec.fill(myP.getUnrollRows()) {
    new OCMRequest(myP.getInBits(), log2Up(myP.thresholdMemDepth)).asOutput
  }
  val thr_rsp = Vec.fill(myP.getUnrollRows()) {
    new OCMResponse(myP.getInBits() * myP.thresholdNumber).asInput
  }
  val act_req = Vec.fill(myP.getUnrollRows()) {
    new OCMRequest(myP.getInBits(), log2Up(myP.inputMemAddr)).asOutput
  }
  val act_rsp = Vec.fill(myP.getUnrollRows()) {
    new OCMResponse(myP.getInBits() * myP.getCols()).asInput
  }

  override def cloneType: this.type =
    new ThrTileMemIO(myP).asInstanceOf[this.type]
}

//TODO Assum Able to write one row per request and not in bit serial way
// interface towards result stage
class ThrStageResMemIO(myP: ThrStageParams) extends Bundle {
  val req = Vec.fill(myP.getRows()) {
    new OCMRequest(
      myP.getResBitWidth() * myP.getCols(), log2Up(myP.resMemAddr)
    ).asOutput
  }

  override def cloneType: this.type =
    new ThrStageResMemIO(myP).asInstanceOf[this.type]
}

class ThrStage(val myP: ThrStageParams) extends Module {
  val io = new Bundle {
    val start = Bool(INPUT) // hold high while running
    val done = Bool(OUTPUT) // high when done until start=0
    val cfg = new ThrStageCfgIO()
    val ctrl = new ThrStageCtrlIO(myP).asInput
    val res = new ThrStageResMemIO(myP)
    val inMemory = new ThrTileMemIO(myP)
  }

  //TODO: ASSUMPTION: fetch from the bram the whole matrix and then start

  val thu = Module(new ThresholdingUnit(myP.thuParams)).io
  //ASSUMING THIS PARAM
  val seqgen = Module(new SequenceGenerator(myP.resMemAddr)).io

  seqgen.init := UInt(0)
  seqgen.count := UInt(myP.getUnrollRows()) //io.csr.numTiles
  seqgen.step := UInt(1) //UInt(myP.tileMemAddrUnit)
  seqgen.start := io.start
  seqgen.seq.ready := Bool(true)

  //never write into that memory
  for (i ← 0 until myP.getUnrollRows()) {
    io.inMemory.act_req(i).writeEn := Bool(false)
    io.inMemory.act_req(i).writeData := UInt(0)
    io.inMemory.act_req(i).addr := UInt(i)
    io.inMemory.thr_req(i).writeEn := Bool(false)
    io.inMemory.thr_req(i).writeData := UInt(0)
    io.inMemory.thr_req(i).addr := UInt(i)
  }

  //ASSUMING that my data are available as long as I need
  //Not convinced at all
  for (i ← 0 until myP.getUnrollRows()) {
    for (j ← 0 until myP.getUnrollCols()) {
      thu.inputMatrix.bits.i(i)(j) := io.inMemory.act_rsp(i).readData(myP.getInBits() * (1 + j) - 1, myP.getInBits() * j)
    }
    for (j ← 0 until myP.thresholdNumber)
      thu.thInterf.thresholdData(i)(j) := io.inMemory.thr_rsp(i).readData(myP.getInBits() * (1 + j) - 1, myP.getInBits() * j)
  }

  //time to write is the latency of the unit + time to write all the outputs
  val time_to_write = myP.thuParams.getLatency() + myP.getUnrollRows()
  val end = ShiftRegister(thu.outputMatrix.valid, time_to_write)
  io.done := end

  for (i ← 0 until myP.getRows())
    for (j ← 0 until myP.getCols()) {
      io.res.req(i).writeEn := thu.outputMatrix.valid
      io.res.req(i).addr := UInt(i)
      io.res.req(i).writeData := UInt(0, width = myP.getResBitWidth() * myP.getCols()) | (thu.outputMatrix.bits.o(i)(j) << j)
    }

}
