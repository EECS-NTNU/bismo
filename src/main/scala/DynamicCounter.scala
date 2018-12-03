// Author:  Davide Conficconi
// Date: 25/09/2018
// Revision: 0

package bismo
import Chisel._

// Counter that dynamically counts till the count max value
class DynamicCounterParams(
  val maxPrecision: Int = 32) extends PrintableParam {

  def headersAsList(): List[String] = {
    return List("MaxInputBitPrecision")
  }

  def contentAsList(): List[String] = {
    return List(maxPrecision).map(_.toString)
  }
}

class DynamicCounter(val p: DynamicCounterParams) extends Module {
  val io = new Bundle {
    val countMax = UInt(INPUT, width = p.maxPrecision)
    val inc = Bool(INPUT)
    val restart = Bool(INPUT)
    val currentValue = UInt(OUTPUT, width = p.maxPrecision)

  }
  val myReg = Reg(init = UInt(0, width = p.maxPrecision))

  when(io.restart || ((myReg === io.countMax - UInt(1)) && io.inc)) {
    myReg := UInt(0, width = p.maxPrecision)
  }.elsewhen(io.inc) {
    myReg := myReg + UInt(1, width = p.maxPrecision)
  }.otherwise {
    myReg := myReg
  }

  io.currentValue := myReg

}
