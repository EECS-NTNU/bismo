// Author:  Davide Conficconi
// Date: 20/08/2018
// Revision: 0
package bismo
import Chisel.{Bool, _}
import fpgatidbits.ocm._

// The Thresholding Unit quantizes an input matrix,
// with a rolling factor or a complete unrolled version
// structurally, it is a compare-popcount datapath.
//TODO: Asssumption: the threshold matrix has one row of threshold per a whole row of input matrix
//TODO Should we handle a quantization vector per element fo the matrix, thus a MxNxT matrix?

class ThresholdingUnitParams(
   // building block params
   val thBBParams : ThresholdingBuildingBlockParams,
  // input bit precision
  val inputBitPrecision : Int = 32,
  // maximum output bit precision
  val maxOutputBitPrecision : Int = 4,
  //Dm number of rows in the input/output matrix 
  val matrixRows : Int,
  //Dn number of columns in the input/output matrix
  val matrixColumns : Int,
  // unrolling factor
  //unrolling in the dimension of the popcount thus the parallel thresholds
  val unrollingFactorOutputPrecision : Int = 15,
  //unrolling factor for the input matrix rows
  val unrollingFactorRows : Int = 1,
  //unrolling factor for the columns of the input matrix
  val unrollingFactorColumns : Int = 1
) extends PrintableParam {

  //check parameters consistency
  //TODO support to Row Roll and different threshold rolling factor
  val maxThresholdNumber : Int = scala.math.pow(2,maxOutputBitPrecision).toInt - 1
  Predef.assert((unrollingFactorOutputPrecision == 1) || (unrollingFactorOutputPrecision == maxThresholdNumber))
  Predef.assert(unrollingFactorRows == matrixRows)
  Predef.assert(unrollingFactorColumns == matrixColumns)
  //how many threshold
    // threshold memory width (how many output bits)
  val thresholdMemWidth : Int = inputBitPrecision * maxThresholdNumber

  val thresholdLatency : Int = maxThresholdNumber / unrollingFactorOutputPrecision

  val rowLatency : Int = matrixRows - unrollingFactorRows

  val colsLatency : Int = matrixColumns - unrollingFactorColumns
  // TODO the TBB has a latency of 1 + 2 for registers in the pipeline + the rolling factors
  val totLatency: Int = thresholdLatency + rowLatency + colsLatency

  def getLatency() : Int = {
    return totLatency
  }
  def getMemWidth() : Int  ={
    return thresholdMemWidth
  }
  def headersAsList(): List[String] = {
    return List("inputBitPrecision", "maxOutputBitPrecision", "matrixRows", "matrixColumns", "thresholdMemWidth", "Unit Latency" ,"unrollingOutPrecision", "UnrollingRows", "UnrollingColumns")
  }

  def contentAsList(): List[String] = {
    return List(inputBitPrecision, maxOutputBitPrecision, matrixRows, matrixColumns, getMemWidth(), getLatency() ,unrollingFactorOutputPrecision, unrollingFactorRows, unrollingFactorColumns).map(_.toString)
  }
}

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
  val thresholdCount = UInt(INPUT, width = p.unrollingFactorOutputPrecision)
  val thresholdData = Vec.fill(p.matrixRows){Vec.fill(p.unrollingFactorOutputPrecision){Bits(INPUT, width = p.inputBitPrecision)}}

  override def cloneType: this.type =
    new ThresholdingInputThresholdIO(p).asInstanceOf[this.type]
}

class ThresholdingUnit(val p: ThresholdingUnitParams) extends Module {
  val io = new Bundle {
    val inputMatrix = Decoupled(new  ThresholdingInputMatrixIO(p)).flip
    val outputMatrix = Decoupled(new ThresholdingOutputMatrixIO(p))
    val thInterf = new ThresholdingInputThresholdIO(p)  

  }
  //TODO ASSUMPTION: fully unrolled matrix as input while rolled threshold matrix
  val inData = Vec.fill(p.unrollingFactorRows){Vec.fill(p.unrollingFactorColumns){UInt()} }

  val outData = Vec.fill(p.unrollingFactorRows){Vec.fill(p.unrollingFactorColumns){UInt()} }
  val outValid = Bool()

  val thData = Vec.fill(p.unrollingFactorRows){Vec.fill(p.unrollingFactorOutputPrecision){UInt()}}

  val thuBB = Vec.fill(p.unrollingFactorRows){Vec.fill(p.unrollingFactorColumns){Module(new ThresholdingBuildingBlock(p.thBBParams)).io}}

  for(i <- 0 until p.unrollingFactorRows )
    for(j <- 0 until p.unrollingFactorColumns )
      inData(i)(j) := io.inputMatrix.bits.i(i)(j)

  //fill the vector of thresholds with the input thresholds
  for (i <- 0 until p.unrollingFactorRows)
    for(j <- 0 until p.unrollingFactorOutputPrecision)
      thData(i)(j) := io.thInterf.thresholdData(i)(j)

  for (i <- 0 until p.unrollingFactorRows)
    for (j <- 0 until p.unrollingFactorColumns)
      outData(i)(j) := thuBB(i)(j).outValue

  //count the clock cycle the unit needs to compute the quantization
  val thCounter = Reg(init = UInt(0, width = log2Up(p.thresholdLatency) + 1 ))

  val iterationFactor = thCounter*UInt(p.unrollingFactorOutputPrecision)


  // If rolled then count till the run-time number otherwise is 1 (quantize no more than that width)
  val runTimeLatency = if(p.unrollingFactorOutputPrecision == 1) {io.thInterf.thresholdCount} else { UInt(1, width = log2Up(p.thresholdLatency))}
  // More threhsolds than the really needed will be filtered with this mask
  val input_mask = Reg(init = UInt(0, width = p.unrollingFactorOutputPrecision), next = io.thInterf.thresholdCount )

  val sInit::sThRoll::sEnd::Nil = Enum(UInt(),3)
  val unitState = Reg(init = sInit)
  //TODO FSM coherent with the rolling factor in the matrix dimensions
  when(unitState === sInit){
    for (i <- 0 until p.unrollingFactorRows)
      for (j <- 0 until p.unrollingFactorColumns)
        for(k <- 0 until p.unrollingFactorOutputPrecision)
        {
          thuBB(i)(j).inVector(k) := UInt(0)
          thuBB(i)(j).thVector(k) := UInt(0)
          thuBB(i)(j).clearAcc := Bool(false)
        }
    outValid := Bool(false)
    when(io.inputMatrix.valid){
      unitState := sThRoll
    }

  }.elsewhen(unitState === sThRoll){
    //ASSUMPTION: fully unrolled matrix rolling in the thresholds
    outValid := Bool(false)
    for (i <- 0 until p.unrollingFactorRows)
      for (j <- 0 until p.unrollingFactorColumns )
        for (k <- 0 until p.unrollingFactorOutputPrecision) {
            thuBB(i)(j).inVector(k) := inData(i)(j) & Fill(p.inputBitPrecision,input_mask(k))
            thuBB(i)(j).thVector(k) := thData(i)(k) & Fill(p.inputBitPrecision,input_mask(k))
          /*************DEBUG PRINT*************/
          //Debugger.log("[HW: THU] InData value "+UInt(i)+", "+UInt(j)+": "+inData(i)(j)+"\n",1)
          //Debugger.log("[HW: THU] ThData value "+UInt(i)+", "+UInt(k)+": "+thData(i)(k)+"\n",1)
          //printf("[HW: THU] InData value %d, %d: %d\n", UInt(i), UInt(j),inData(i)(j) )
          //printf("[HW: THU] ThData value %d, %d: %d\n", UInt(i), UInt(k),thData(i)(k) )
          //printf("[HW: THU] Clear Acc value %d, %d: %d\n", UInt(i), UInt(k), thuBB(i)(j).clearAcc)
          
        }
    thCounter := thCounter + UInt(1)
    when(thCounter ===  runTimeLatency - UInt(1)/*UInt(p.thresholdLatency - 1)*/ ){
      unitState := sEnd
    }

  }.elsewhen(unitState === sEnd){
    outValid := Bool(true)
    when(io.outputMatrix.ready){
      /*************DEBUG PRINT*************/
//      for (i <- 0 until p.unrollingFactorRows)
//        for (j <- 0 until p.unrollingFactorColumns){
//          //Debugger.log("[HW: THU] Out Data value "+UInt(i)+", "+UInt(j)+": "+outData(i)(j)+"\n",1)
//
//          printf("[HW: THU] Out Data value %d, %d: %d\n", UInt(i), UInt(j),outData(i)(j) )
//        }
      /*************    END    *************/
      thCounter := UInt(0)
      for (i <- 0 until p.unrollingFactorRows)
        for (j <- 0 until p.unrollingFactorColumns)
          thuBB(i)(j).clearAcc := Bool(true)
      unitState := sInit
      outValid := Bool(false)
    }
  }.otherwise{
    //Default value for graceful dead
    for (i <- 0 until p.unrollingFactorRows)
      for (j <- 0 until p.unrollingFactorColumns)
        for(k <- 0 until p.unrollingFactorOutputPrecision )
        {
          thuBB(i)(j).inVector(k) := UInt(0)
          thuBB(i)(j).thVector(k) := UInt(0)
          thuBB(i)(j).clearAcc := Bool(false)
        }
    outValid := Bool(false)
  }

  io.outputMatrix.valid := outValid
  io.outputMatrix.bits.o := outData
}