
// Author:  Davide Conficconi
// Date: 17/10/2018
// Revision: 0

package bismo

import Chisel._
import fpgatidbits.ocm._
import fpgatidbits.streams._

class P2SKernelParams (
                        val maxInBw : Int = 8,
                        val nInElemPerWord : Int = 8,
                        val outStreamSize : Int = 64
) extends PrintableParam {

 def headersAsList(): List[String] = {
   return List("M","N","O")
 }
 def contentAsList() : List[String] = {
    return List(maxInBw,nInElemPerWord,outStreamSize).map(_.toString)
  }
}


class P2SKernelCtrlIO(myP: P2SKernelParams) extends PrintableBundle{
  val dramBaseAddrSrc = UInt(width = 32)
  val dramBaseAddrDst = UInt(width = 32)
  val matrixRows = UInt(width = 32)
  val matrixCols = UInt(width = 32)
  val actualInBw = UInt(width = log2Up(myP.maxInBw))

  override def cloneType(): this.type =
    new P2SKernelCtrlIO(myP).asInstanceOf[this.type]
  val printfStr = "DRAM base src addr: %d, dst addr: %d, Matrix size: %d, %d, with current precision of %d\n"
  val printfElems = {()=> Seq(dramBaseAddrSrc, dramBaseAddrDst, matrixRows, matrixCols, actualInBw)}
}

class P2SKernel (myP: P2SKernelParams) extends Module {
  val io = new Bundle{
      val ctrl = new P2SKernelCtrlIO(myP: P2SKernelParams)
      val inputStream =  Decoupled((UInt( width = myP.maxInBw * myP.nInElemPerWord)).asInput()).flip()
      val outStream = Decoupled(UInt(width = myP.outStreamSize)).asOutput()
  }

  val operands = Vec.fill(myP.nInElemPerWord){Reg(init = UInt(0, width=myP.maxInBw)}

  when(io.inputStream.valid){
    io.outStream.valid := Bool(true)
  }.otherwise{
    io.outStream.valid := Bool(false)
  }
  for(i <- 0 until myP.nInElemPerWord)
    operands(i) := io.inputStream.bits( myP.maxInBw * (1 + i) - 1, myP.maxInBw * i)

  for(i <- 0 until myP.nInElemPerWord-1)
    io.outStream := Cat(operands(i) + operands(i+1))
}
