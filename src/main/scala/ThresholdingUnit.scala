// Author:  Davide Conficconi
// Date: 20/08/2018
// Revision: 0
package bismo
import Chisel._
import fpgatidbits.ocm._

// The Thresholding Unit quantizes an input matrix,
// with a rolling factor or a complete unrolled version
// structurally, it is a compare-popcount datapath.

class ThresholdingUnitParams(
  // input bit precision
  val inputBitPrecision : Int = 32,
  // maximum output bit precision
  val maxOutputBitPrecision : Int = 4,
  //Dm number of rows in the input/output matrix 
  val matrixRows : Int,
  //Dn number of columns in the input/output matrix
  val matrixColumns : Int,
  // threshold memory depth (how many entries, address space)
  val thresholdMemDepth : Int,
  // threshold memory width (how many output bits)
  // MY WORRIES: Should it be matrixColumns-1 * maxOutputBitPrecision?
  val thresholdMemWidth : Int,
  // unrolling factor
  // MY WORRIES: Should we guarantee the consistency of this parameter somehow?
  val unrollingFactor : Int
) extends PrintableParam {
  // internal pipeline registers inside DPU
  //val myLatency = if(useVhdlCompressor){5 + extraPipelineRegs} else {6 + extraPipelineRegs}
  // latency of instantiated PopCountUnit
  //val popcountLatency: Int = pcParams.getLatency()
  // return total latency
  //def getLatency(): Int = {
  //  return myLatency + popcountLatency
  //}
  def headersAsList(): List[String] = {
    return List("inputBitPrecision", "maxOutputBitPrecision", "matrixRows", "matrixColumns", "thresholdMemDepth", "thresholdMemWidth", "unrollingFactor")
  }

  def contentAsList(): List[String] = {
    return List(inputBitPrecision, maxOutputBitPrecision, matrixRows, matrixColumns,thresholdMemDepth, thresholdMemWidth, unrollingFactor).map(_.toString)
  }
}

//MY WORRIES: Handle DecoupledIO?
class ThresholdingInputMatrixIO (val p: ThresholdingUnitParams) extends Bundle{
  val i = Vec.fill(p.matrixRows){Vec.fill(p.matrixColumns){Bits(INPUT, width = p.inputBitPrecision)}}
  val iValid = Bool(INPUT)
}

class ThresholdingOutputMatrixIO (val p: ThresholdingUnitParams) extends Bundle{
  val o = Vec.fill(p.matrixRows){Vec.fill(p.matrixColumns){Bits(OUTPUT, width = p.maxOutputBitPrecision)}}
  val oValid = Bool(OUTPUT)
}

class ThresholdingInputThresholdIO (val p: ThresholdingUnitParams) extends Bundle{
  //MY WORRIES: should we handle at IO level the unrolling or internal management?
  // In case of IO we should change this interface
  //memory interfacing
  val thresholdVector = new OCMResponse(p.thresholdMemWidth).asInput
  val thresholdRequest = new OCMRequest(p.thresholdMemWidth,log2Up(p.thresholdMemDepth)).asOutput 
}

class ThresholdingUnit(val p: ThresholdingUnitParams) extends Module {
  val io = new Bundle {
    val inputMatrix = new ThresholdingInputMatrixIO(p)
    val outputMatrix = new ThresholdingOutputMatrixIO(p)
    val thresholdsInterf = new ThresholdingInputThresholdIO(p)  
  }
  //A register for temporary matrix storage and valid signal 
  val inRegData = Vec.fill(p.matrixRows){Vec.fill(p.matrixColumns){Reg(outType = UInt(width =  p.inputBitPrecision))}}
  val inRegValid = Reg(init = Bool(false), next = io.inputMatrix.iValid)
  
  for(i <- 0 to p.matrixRows - 1)
    for(j <- 0 to p.matrixColumns - 1 )
      inRegData(i)(j) := io.inputMatrix.i(i)(j)


  val outRegData = Vec.fill(p.matrixRows){Vec.fill(p.matrixColumns){Reg(outType = UInt(width = p.maxOutputBitPrecision))}}



  //threshold management
  val thRegData = Vec.fill(p.matrixColumns){Reg(outType = UInt(width = p.maxOutputBitPrecision))}
  //val thAddressWire = UInt(0, width = log2Up(p.thresholdMemDepth))

  //fill the vector of thresholds with the input thresholds
  for (i <- 0 to p.matrixColumns - 1)
    thRegData(i):= io.thresholdsInterf.thresholdVector.readData(((i+1)*p.maxOutputBitPrecision)-1,i*p.maxOutputBitPrecision)

  val addrThReg = Reg(init = UInt(0, width = log2Up(p.thresholdMemDepth)))

  //Dm/rows clock cycle required for complete a matrix quantization.
  val busyUnit = Reg(init = Bool(false))
  val rowCounter = Reg(init = UInt(width = log2Up(p.matrixRows) + 1))

  //setting the address request for the Threshold memory
  io.thresholdsInterf.thresholdRequest.addr := addrThReg
  io.thresholdsInterf.thresholdRequest.writeEn := Bool(false)

  //valid input matrix and not currently busy unit
  when(inRegValid && !(busyUnit)){
    busyUnit := Bool(true)
    rowCounter := UInt(0)
    addrThReg := UInt(0)
  }

  val comparisonRes = Vec.fill(p.matrixColumns){UInt(width=p.maxOutputBitPrecision)}



  for (i <- 0 to p.matrixColumns - 1)
    comparisonRes(i) := (inRegData(rowCounter)(i) >= thRegData(i))

  when(busyUnit && (rowCounter <= UInt(p.matrixRows)) ){
    rowCounter := rowCounter + UInt(1)
    addrThReg := addrThReg + UInt(1)
    for (j <- 0 to p.matrixColumns - 1)
      outRegData(rowCounter)(j):= PopCount(comparisonRes(j))
  }.elsewhen(busyUnit && (rowCounter === UInt(p.matrixRows)) ){
    busyUnit := Bool(false)
  }

  io.outputMatrix.o := outRegData
}