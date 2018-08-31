// Author:  Davide Conficconi
// Date: 23/08/2018
// Revision: 0
package bismo
import Chisel._

class ThresholdingBuildingBlockParams(
	//input precision, how many bits
	val inPrecision : Int = 32,
	// how many input- threshold the unit should perform in paralle
	val popcountUnroll : Int = 1,
	//output bit precision
	val outPrecision : Int = 1
) extends PrintableParam {

  //check parameters consistency
  Predef.assert(popcountUnroll == (scala.math.pow(2,outPrecision)toInt) )

  def headersAsList(): List[String] = {
    return List("InputBitPrecision", "OutputBitPrecision", "PopcountUnrollingFactor")
  }

  def contentAsList(): List[String] = {
    return List(inPrecision, popcountUnroll, outPrecision).map(_.toString)
  }
}

class ThresholdingBuildingBlock(val p: ThresholdingBuildingBlockParams) extends Module {
  val io = new Bundle {
  	val inVector = Vec.fill(p.popcountUnroll){Bits(INPUT, width = p.inPrecision)}
  	val thVector = Vec.fill(p.popcountUnroll){Bits(INPUT, width = p.inPrecision)}
  	val outValue = Bits(OUTPUT, width = p.outPrecision)
		val clearAcc = Bool(INPUT)
  	}

  	val compareVector = Vec.fill(p.popcountUnroll){Bool()}
  	for(i <- 0 to p.popcountUnroll - 1 )
  		compareVector(i) := io.inVector(i) > io.thVector(i)
  	val popcountVector = UInt()
    popcountVector := PopCount(compareVector)
  	//MY WORRIES: should it be  reinitialized for every input? Should we handle this at the upper level
		val next_value = UInt()
	  val outReg = Reg(init = UInt(0, width = p.outPrecision), next = next_value)
    when(io.clearAcc){
      next_value := UInt(0)
    }.otherwise{
      next_value := popcountVector + outReg
    }
  	io.outValue := outReg


}