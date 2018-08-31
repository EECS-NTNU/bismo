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
  // unrolling factor
  // MY WORRIES: Should we guarantee the consistency of this parameter somehow?
  //unrolling in the dimension of the popcount
  val unrollingFactorOutputPrecision : Int = 16,
  //unrolling factor for the input matrix rows
  val unrollingFactorRows : Int = 1,
  //unrolling factor for the columns of the input matrix
  val unrollingFactorColumns : Int = 1
) extends PrintableParam {

  //check parameters consistency
  Predef.assert(unrollingFactorRows == matrixRows)
  Predef.assert(unrollingFactorColumns == matrixColumns)
  Predef.assert(unrollingFactorOutputPrecision == (scala.math.pow(2,maxOutputBitPrecision)toInt))

    // threshold memory width (how many output bits)
  // MY WORRIES: Should it be matrixColumns-1 * maxOutputBitPrecision?
  val thresholdMemWidth : Int = inputBitPrecision * (scala.math.pow(2,maxOutputBitPrecision)toInt)

  val thersholdLatency : Int = (scala.math.pow(2,maxOutputBitPrecision)toInt) / unrollingFactorOutputPrecision

  val rowLatency : Int = matrixRows / unrollingFactorRows

  val colsLatency : Int = matrixColumns / unrollingFactorColumns
  // internal pipeline registers inside DPU
  //val myLatency = if(useVhdlCompressor){5 + extraPipelineRegs} else {6 + extraPipelineRegs}
  // latency of instantiated PopCountUnit
  //val popcountLatency: Int = pcParams.getLatency()
  // return total latency
  //def getLatency(): Int = {
  //  return myLatency + popcountLatency
  //}
  def headersAsList(): List[String] = {
    return List("inputBitPrecision", "maxOutputBitPrecision", "matrixRows", "matrixColumns", "thresholdMemDepth", "thresholdMemWidth", "unrollingOutPrecision", "UnrollingRows", "UnrollingColumns")
  }

  def contentAsList(): List[String] = {
    return List(inputBitPrecision, maxOutputBitPrecision, matrixRows, matrixColumns,thresholdMemDepth, thresholdMemWidth, unrollingFactorOutputPrecision, unrollingFactorRows, unrollingFactorColumns).map(_.toString)
  }
}

//MY WORRIES: Handle DecoupledIO?
class ThresholdingInputMatrixIO (val p: ThresholdingUnitParams) extends Bundle{
  val i = Vec.fill(p.matrixRows){Vec.fill(p.matrixColumns){Bits(INPUT, width = p.inputBitPrecision)}}

  override def cloneType: this.type =
    new ThresholdingInputMatrixIO(p).asInstanceOf[this.type]
}

class ThresholdingOutputMatrixIO (val p: ThresholdingUnitParams) extends Bundle{
  val o = Vec.fill(p.matrixRows){Vec.fill(p.matrixColumns){Bits(OUTPUT, width = p.maxOutputBitPrecision)}}
  
  override def cloneType: this.type =
    new ThresholdingOutputMatrixIO(p).asInstanceOf[this.type]
}

class ThresholdingInputThresholdIO (val p: ThresholdingUnitParams) extends Bundle{
  //MY WORRIES: should we handle at IO level the unrolling or internal management?
  // In case of IO we should change this interface
  //memory interfacing
  //val thresholdVector = new OCMResponse(p.thresholdMemWidth).asInput
  //val thresholdRequest = new OCMRequest(p.thresholdMemWidth,log2Up(p.thresholdMemDepth)).asOutput 
  val thresholdAddr = UInt(OUTPUT, width = log2Up(p.thresholdMemDepth))
  val thresholdData = Bits(INPUT, width = p.thresholdMemWidth)//Vec.fill(p.matrixRows){Bits(INPUT, width = p.inputBitPrecision)}

  override def cloneType: this.type =
    new ThresholdingInputThresholdIO(p).asInstanceOf[this.type]
}

class ThresholdingUnit(val p: ThresholdingUnitParams) extends Module {
  val io = new Bundle {
    val inputMatrix = Decoupled(new  ThresholdingInputMatrixIO(p)).flip
    val outputMatrix = Decoupled(new ThresholdingOutputMatrixIO(p))
    val thInterf = new ThresholdingInputThresholdIO(p)  

  }
  //A register for temporary matrix storage and valid signal 
  val inRegData = Vec.fill(p.matrixRows){Vec.fill(p.matrixColumns){Reg(outType = UInt(width =  p.inputBitPrecision))} }
  val inRegValid = Reg(init = Bool(false), next = io.inputMatrix.valid)
  
  for(i <- 0 to p.matrixRows - 1)
    for(j <- 0 to p.matrixColumns - 1 )
      inRegData(i)(j) := io.inputMatrix.bits.i(i)(j)


  val outRegData = Vec.fill(p.matrixRows){Vec.fill(p.matrixColumns){Reg(outType = UInt(width = p.maxOutputBitPrecision))} }

  //threshold management
  //MY WORRIES: change with input precision
  val thRegData = Vec.fill(p.unrollingFactorOutputPrecision){Reg(outType = UInt(width = p.inputBitPrecision))}

  //fill the vector of thresholds with the input thresholds
  for (i <- 0 to p.unrollingFactorOutputPrecision - 1)
    thRegData(i):= io.thInterf.thresholdData(((i+1)*p.maxOutputBitPrecision)-1,i*p.maxOutputBitPrecision)

  val addrThReg = Reg(init = UInt(0, width = log2Up(p.thresholdMemDepth)))


  val d = new ThresholdingBuildingBlockParams(p.inputBitPrecision , p.unrollingFactorOutputPrecision, p.maxOutputBitPrecision)

  val thuBB = Vec.fill(p.unrollingFactorRows){Vec.fill(p.unrollingFactorColumns){Module(new ThresholdingBuildingBlock(d)).io}}

  //TODO : handle partial unroll for rows columns and threshold

  val ready = Reg(init = Bool(true))
  val busy = Reg(init = Bool(false), next = !(ready))

  io.inputMatrix.ready := ready
  when(ready && io.inputMatrix.valid){
    ready := Bool(false)
  }

  val counter = Reg(init = UInt(0))

  when(busy && Bool(counter == UInt(0))){
    for(i <- 0 to p.matrixRows - 1)
      for(j <- 0 to p.matrixColumns - 1)
        for(k <- 0 to p.unrollingFactorOutputPrecision - 1){
          thuBB(i)(j).inVector(k) := inRegData(i)(j)
          thuBB(i)(j).thVector(k) := thRegData(k)
        }
    counter := counter + UInt(1)

  }.otherwise{
    for(i <- 0 to p.matrixRows - 1)
      for(j <- 0 to p.matrixColumns - 1)
        for(k <- 0 to p.unrollingFactorOutputPrecision - 1){
          thuBB(i)(j).inVector(k) := UInt(0)
          thuBB(i)(j).thVector(k) := UInt(0)
        }
    counter := UInt(0)
    ready := Bool(true)
  }

 for(i <- 0 to p.matrixRows - 1)
    for(j <- 0 to p.matrixColumns - 1)
      outRegData(i)(j) := thuBB(i)(j).outValue
  
  io.thInterf.thresholdAddr := addrThReg
  io.outputMatrix.bits.o := outRegData
}