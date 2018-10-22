
// Author:  Davide Conficconi
// Date: 17/10/2018
// Revision: 0

package bismo

import Chisel._
import fpgatidbits.ocm._
import fpgatidbits.streams._
import fpgatidbits.dma._

class P2SKernelParams (
                        val maxInBw : Int = 8,
                        val nInElemPerWord : Int = 8,
                        val outStreamSize : Int = 64,
                        val mrp: MemReqParams,
                        val suparams: SerializerUnitParams
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
  val actualInBw = UInt(width = myP.maxInBw)
  val waitCompleteBytes = UInt(width = 32)

  override def cloneType(): this.type =
    new P2SKernelCtrlIO(myP).asInstanceOf[this.type]
  val printfStr = "DRAM base src addr: %d, dst addr: %d, Matrix size: %d, %d, with current precision of %d\n"
  val printfElems = {()=> Seq(dramBaseAddrSrc, dramBaseAddrDst, matrixRows, matrixCols, actualInBw)}
}

class P2SKernel (myP: P2SKernelParams) extends Module {
  val io = new Bundle{
      val ctrl = new P2SKernelCtrlIO(myP: P2SKernelParams).asInput()
      val dramBaseSrc = UInt(OUTPUT, width = 32)
      val dramBaseDst = UInt(OUTPUT, width = 32)
      val inputStream = Decoupled(UInt( width = myP.maxInBw * myP.nInElemPerWord)).flip()
      val outStream = Decoupled(UInt(width = myP.mrp.dataWidth))
      val start = Bool(INPUT)
      val done = Bool(OUTPUT)
  }

  io.dramBaseDst := io.ctrl.dramBaseAddrDst
  io.dramBaseSrc := io.ctrl.dramBaseAddrSrc

  val operands = Vec.fill(myP.nInElemPerWord){Reg(init = UInt(0, width=myP.maxInBw))}
  val filteredOps = Vec.fill((myP.nInElemPerWord)){UInt(width=myP.maxInBw)}


  io.outStream.valid := ShiftRegister(io.inputStream.valid,1)
  //when(io.outStream.valid){
  //  printf("[HW: P2SKrnl] Input valid \n")
  //}
  for(i <- 0 until myP.nInElemPerWord) {
    operands(i) := io.inputStream.bits(myP.maxInBw * (1 + i) - 1, myP.maxInBw * i)
    filteredOps(i) := operands(i) & io.ctrl.actualInBw
  }
   val sumVec = Vec.fill(myP.nInElemPerWord/2){UInt()}

  val serUnit = Module ( new SerializerUnit(myP.suparams)).io

  serUnit.counterValue := io.ctrl.actualInBw

  for(i <- 0 until myP.nInElemPerWord){
    serUnit.input(0)(i) := filteredOps(i)
  }


  // FSM logic for control
  val sSUReady :: sSUProcessing :: Nil = Enum(UInt(), 2)
  val regState = Reg(init = UInt(sSUReady))



  val suStart = Reg(init = Bool(true))

  when(io.inputStream.valid ){
    suStart := Bool(true)
  }
  serUnit.start := suStart

  io.inputStream.ready := io.start & serUnit.out.valid

  for(i <- 0 until myP.nInElemPerWord/2 ) {
    sumVec(i) := filteredOps(i*2) + filteredOps(i*2 + 1 )
   // printf("[HW: P2SKrnl] Adding %d + %d\n", filteredOps(i), filteredOps(myP.nInElemPerWord - 1 - i) )

  }

   io.outStream.bits := sumVec.asUInt()
/*
  when(io.outStream.valid){
      printf("[HW: P2SKrnl] Output valid Data %d\n", io.outStream.bits)

  }
*/

}
