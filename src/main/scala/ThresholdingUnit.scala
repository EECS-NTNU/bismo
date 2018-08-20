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
  val inRegData = Vec.fill(p.matrixRows){Vec.fill(p.matrixColumns){Reg(outType = p.inputBitPrecision)}}
  val inRegValid = Reg(init = Bool(false), next = io.inputMatrix.iValid)
  // NEED TO BIND THE REGISTER WITH THIS WIRE OR WRITE BETTER THE ABOVE REGISTER
  for(int i = 0; i<p.matrixRows;i++)
    for(int j =0; j<p.matrixColumns; j++)
      inRegData(i)(j) := io.inputMatrix.i(i)(j)


  val outRegData = Vec.fill(p.matrixRows){Vec.fill(p.matrixColumns){Reg(outType = UInt(width = p.maxOutputBitPrecision))}}



  //threshold management
  val thRegData = Vec.fill(p.matrixColumns){Reg(outType = UInt(width = p.maxOutputBitPrecision))}
  val thAddressWire = UInt(width = log2Up(p.thresholdMemDepth))

  //fill the vector of thresholds with the input thresholds
  for (int i = 0; i<p.matrixColumns; i++)
    thRegData(i):= io.thresholdsInterf.thresholdVector(((i+1)*p.maxOutputBitPrecision)-1,i*p.maxOutputBitPrecision)

  val addrThReg = Reg(init = UInt(0, width = log2Up(p.thresholdMemDepth)), next = thAddressWire)

  //Dm/rows clock cycle required for complete a matrix quantization.
  val busyUnit = Reg(init = Bool(false))
  val rowCounter = Reg(init = UInt(width = log2Up(p.matrixRows) + 1))

  

  //valid input matrix and not currently busy unit
  when(inRegValid && !(busyUnit)){
    busyUnit := Bool(true)
    rowCounter := UInt(0)
    thAddressWire := UInt(0)
  }

  val comparisonRes = Vec.fill(p.matrixColumns){UInt(width=p.maxOutputBitPrecision)}



  for (int i = 0; i<p.matrixColumns; i++)
  comparisonRes := inRegData >= thRegData(i) 
  when(busyUnit && (rowCounter <= p.matrixRows)){
    rowCounter := rowCounter + 1
    thAddressWire := thAddressWire + 1
    for (int j = 0; j < p.matrixColumns; j++)
      outRegData(rowCounter)(j):= PopCount(comparisonRes(j))
  }.elsewhen(busyUnit && (rowCounter === p.matrixRows)){
    busyUnit := Bool(false)
  }

  io.outputMatrix.o := outRegData
}