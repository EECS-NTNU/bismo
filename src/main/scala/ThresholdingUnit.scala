// Author:  Davide Conficconi
// Date: 20/08/2018
// Revision: 0
package bismo
import Chisel.{ Bool, _ }
import fpgatidbits.ocm._

// The Thresholding Unit quantizes an input matrix,
// with a rolling factor or a complete unrolled version
// structurally, it is a compare-popcount datapath.
//TODO: Asssumption: the threshold matrix has one row of threshold per a whole row of input matrix
//TODO Should we handle a quantization vector per element fo the matrix, thus a MxNxT matrix?

class ThresholdingUnitParams(
    // building block params
    val thBBParams: ThresholdingBuildingBlockParams,
    // input bit precision
    val inputBitPrecision: Int = 32,
    // maximum output bit precision
    val maxOutputBitPrecision: Int = 4,
    //Dm number of rows in the input/output matrix
    val matrixRows: Int,
    //Dn number of columns in the input/output matrix
    val matrixColumns: Int,
    // threshold memory depth (how many entries, address space)
    val thresholdMemDepth: Int,
    // unrolling factor
    //unrolling in the dimension of the popcount thus the parallel thresholds
    val unrollingFactorOutputPrecision: Int = 16,
    //unrolling factor for the input matrix rows
    val unrollingFactorRows: Int = 1,
    //unrolling factor for the columns of the input matrix
    val unrollingFactorColumns: Int = 1) extends PrintableParam {

  //check parameters consistency
  //TODO support to Row Roll
  Predef.assert(unrollingFactorRows == matrixRows)
  Predef.assert(unrollingFactorColumns == matrixColumns)
  //how many threshold
  val thresholdNumber: Int = scala.math.pow(2, maxOutputBitPrecision).toInt - 1
  // threshold memory width (how many output bits)
  val thresholdMemWidth: Int = inputBitPrecision * thresholdNumber

  val thresholdLatency: Int = thresholdNumber - unrollingFactorOutputPrecision

  val rowLatency: Int = matrixRows - unrollingFactorRows

  val colsLatency: Int = matrixColumns - unrollingFactorColumns
  // TODO the TBB has a latency of 1 + 2 for registers in the pipeline + the rolling factors
  val totLatency: Int = thresholdLatency + rowLatency + colsLatency + 3

  def getLatency(): Int = {
    return totLatency
  }
  def getMemWidth(): Int = {
    return thresholdMemWidth
  }
  def headersAsList(): List[String] = {
    return List("inputBitPrecision", "maxOutputBitPrecision", "matrixRows", "matrixColumns", "thresholdMemDepth", "thresholdMemWidth", "Unit Latency", "unrollingOutPrecision", "UnrollingRows", "UnrollingColumns")
  }

  def contentAsList(): List[String] = {
    return List(inputBitPrecision, maxOutputBitPrecision, matrixRows, matrixColumns, thresholdMemDepth, getMemWidth(), getLatency(), unrollingFactorOutputPrecision, unrollingFactorRows, unrollingFactorColumns).map(_.toString)
  }
}

class ThresholdingInputMatrixIO(val p: ThresholdingUnitParams) extends Bundle {
  val i = Vec.fill(p.matrixRows) { Vec.fill(p.matrixColumns) { Bits(INPUT, width = p.inputBitPrecision) } }

  override def cloneType: this.type =
    new ThresholdingInputMatrixIO(p).asInstanceOf[this.type]
}

class ThresholdingOutputMatrixIO(val p: ThresholdingUnitParams) extends Bundle {
  val o = Vec.fill(p.matrixRows) { Vec.fill(p.matrixColumns) { Bits(OUTPUT, width = p.maxOutputBitPrecision) } }

  override def cloneType: this.type =
    new ThresholdingOutputMatrixIO(p).asInstanceOf[this.type]
}

class ThresholdingInputThresholdIO(val p: ThresholdingUnitParams) extends Bundle {
  val thresholdAddr = UInt(OUTPUT, width = log2Up(p.thresholdMemDepth))
  val thresholdData = Vec.fill(p.matrixRows) { Vec.fill(p.thresholdNumber) { Bits(INPUT, width = p.inputBitPrecision) } }

  override def cloneType: this.type =
    new ThresholdingInputThresholdIO(p).asInstanceOf[this.type]
}

class ThresholdingUnit(val p: ThresholdingUnitParams) extends Module {
  val io = new Bundle {
    val inputMatrix = Decoupled(new ThresholdingInputMatrixIO(p)).flip
    val outputMatrix = Decoupled(new ThresholdingOutputMatrixIO(p))
    val thInterf = new ThresholdingInputThresholdIO(p)

  }
  //A register for temporary matrix storage and valid signal
  //TODO : should become a BRAM storage
  val inRegData = Vec.fill(p.matrixRows) { Vec.fill(p.matrixColumns) { Reg(outType = UInt(width = p.inputBitPrecision)) } }
  val inRegValid = Reg(init = Bool(false), next = io.inputMatrix.valid)

  val outRegData = Vec.fill(p.matrixRows) { Vec.fill(p.matrixColumns) { Reg(outType = UInt(width = p.maxOutputBitPrecision)) } }
  val outRegValid = Reg(init = Bool(false))

  val thRegData = Vec.fill(p.matrixRows) { Vec.fill(p.thresholdNumber) { Reg(outType = UInt(width = p.inputBitPrecision)) } }

  val thuBB = Vec.fill(p.unrollingFactorRows) { Vec.fill(p.unrollingFactorColumns) { Module(new ThresholdingBuildingBlock(p.thBBParams)).io } }

  for (i ← 0 until p.matrixRows)
    for (j ← 0 until p.matrixColumns)
      inRegData(i)(j) := io.inputMatrix.bits.i(i)(j)

  //fill the vector of thresholds with the input thresholds
  for (i ← 0 until p.matrixRows)
    for (j ← 0 until p.thresholdNumber)
      thRegData(i)(j) := io.thInterf.thresholdData(i)(j)

  for (i ← 0 until p.matrixRows)
    for (j ← 0 until p.matrixColumns)
      outRegData(i)(j) := thuBB(i)(j).outValue

  val addrThReg = Reg(init = UInt(0, width = log2Up(p.thresholdMemDepth)))

  //count the clock cycle the unit needs to compute the quantization
  val thCounter = Reg(init = UInt(0, width = log2Up(p.thresholdLatency) + 1))

  val iterationFactor = thCounter * UInt(p.unrollingFactorOutputPrecision)
  val sInit :: sThRoll :: sEnd :: Nil = Enum(UInt(), 3)
  val unitState = Reg(init = sInit)
  when(unitState === sInit) {
    for (i ← 0 until p.matrixRows)
      for (j ← 0 until p.matrixColumns)
        for (k ← 0 until p.unrollingFactorOutputPrecision) {
          thuBB(i)(j).inVector(k) := UInt(0)
          thuBB(i)(j).thVector(k) := UInt(0)
          thuBB(i)(j).clearAcc := Bool(false)
        }
    outRegValid := Bool(false)
    when(inRegValid) {
      unitState := sThRoll
    }

  }.elsewhen(unitState === sThRoll) {
    //ASSUMPTION: fully unrolled matrix rolling in the thresholds
    outRegValid := Bool(false)
    for (i ← 0 until p.matrixRows)
      for (j ← 0 until p.matrixColumns)
        for (k ← 0 until p.unrollingFactorOutputPrecision) {
          thuBB(i)(j).inVector(k) := inRegData(i)(j)
          thuBB(i)(j).thVector(k) := thRegData(i)(iterationFactor + UInt(k))
        }
    thCounter := thCounter + UInt(1)
    when(thCounter === UInt(p.thresholdLatency)) {
      unitState := sEnd
    }

  }.elsewhen(unitState === sEnd) {
    outRegValid := Bool(true)
    thCounter := UInt(0)
    for (i ← 0 until p.matrixRows)
      for (j ← 0 until p.matrixColumns)
        thuBB(i)(j).clearAcc := Bool(true)

    unitState := sInit
  }.otherwise {
    //Default value for graceful dead
    for (i ← 0 until p.matrixRows)
      for (j ← 0 until p.matrixColumns)
        for (k ← 0 until p.unrollingFactorOutputPrecision) {
          thuBB(i)(j).inVector(k) := UInt(0)
          thuBB(i)(j).thVector(k) := UInt(0)
          thuBB(i)(j).clearAcc := Bool(false)
        }
    outRegValid := Bool(false)
  }

  io.outputMatrix.valid := outRegValid
  io.outputMatrix.bits.o := outRegData
}
