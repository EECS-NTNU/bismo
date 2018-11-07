// Author:  Davide Conficconi
// Date: 23/08/2018
// Revision: 0
package bismo
import Chisel._

class ThresholdingBuildingBlockParams(
  //input precision, how many bits
  val inPrecision: Int = 32,
  // how many input- threshold the unit should perform in parallel
  val popcountUnroll: Int = 1,
  //output bit precision
  val outPrecision: Int = 1) extends PrintableParam {

  //check parameters consistency for full unroll
  //Predef.assert(popcountUnroll == (scala.math.pow(2,outPrecision).toInt - 1) )

  def headersAsList(): List[String] = {
    return List("InputBitPrecision", "OutputBitPrecision", "PopcountUnrollingFactor")
  }

  def contentAsList(): List[String] = {
    return List(inPrecision, popcountUnroll, outPrecision).map(_.toString)
  }
}

class ThresholdingBuildingBlock(val p: ThresholdingBuildingBlockParams) extends Module {
  val io = new Bundle {
    val inVector = Vec.fill(p.popcountUnroll) { Bits(INPUT, width = p.inPrecision) }
    val thVector = Vec.fill(p.popcountUnroll) { Bits(INPUT, width = p.inPrecision) }
    val outValue = Bits(OUTPUT, width = p.outPrecision)
    ///DEBUGGING OUTPUT
    //    val tmp = Vec.fill(p.inPrecision){Bits(OUTPUT, width = 1)}
    val clearAcc = Bool(INPUT)
  }

  //TODO comparison between 2c2 numbers an not
  //  def compare2c2(a : UInt, b : UInt) : Bool = {
  //    //same sign
  //    if( a(p.inPrecision-1) == b(p.inPrecision-1)){
  //      return a > b
  //   }//different sign
  //    else
  //      return ~(a>b)
  //
  //  }
  ///////DEBUGGING OUTPUT
  //  for(i<-0 until p.inPrecision)
  //    io.tmp(i) := io.inVector(0)(i)
  ///////////////////

  //intermediate wire combinationally filled with the comparison against input and threshold
  val compareVector = Vec.fill(p.popcountUnroll) { Bool() }
  for (i ← 0 to p.popcountUnroll - 1) {
    compareVector(i) := io.inVector(i) > io.thVector(i) //compare2c2(io.inVector(i),io.thVector(i))
  }

  //popcount for the quantization
  val popcountVector = UInt()
  popcountVector := PopCount(compareVector)
  //feedback loop closure for unit reuse and roll
  val next_value = UInt()
  val outReg = Reg(init = UInt(0, width = p.outPrecision), next = next_value)
  //clear the register whenever needed otherwise increment the popcount
  when(io.clearAcc) {
    next_value := popcountVector
  }.otherwise {
    next_value := popcountVector + outReg
  }
  io.outValue := outReg

}
