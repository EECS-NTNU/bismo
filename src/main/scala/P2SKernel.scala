
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
  val dramBaseAddrSrc = UInt(width = 32) // ALWAYS the address of the current row
  val dramBaseAddrDst = UInt(width = 32) // ALWAYS the addres of b0 - curr row
  val matrixRows = UInt(width = 32)// which number of row currently processing
  val matrixCols = UInt(width = 32)//how many cols
  val actualInBw = UInt(width = myP.maxInBw)
  val waitCompleteBytes = UInt(width = 32)

  override def cloneType(): this.type =
    new P2SKernelCtrlIO(myP).asInstanceOf[this.type]
  val printfStr = "DRAM base src addr: %d, dst addr: %d, Matrix size: %d, %d, with current precision of %d\n"
  val printfElems = {()=> Seq(dramBaseAddrSrc, dramBaseAddrDst, matrixRows, matrixCols, actualInBw)}
}

class P2SKernel (myP: P2SKernelParams) extends Module {
  val io = new Bundle {
    val ctrl = new P2SKernelCtrlIO(myP: P2SKernelParams).asInput()
    val dramBaseSrc = UInt(OUTPUT, width = 32)
    val dramBaseDst = UInt(OUTPUT, width = 32)
    val inputStream = Decoupled(UInt(width = myP.maxInBw * myP.nInElemPerWord)).flip()
    val outStream = Decoupled(UInt(width = myP.mrp.dataWidth))
    val start = Bool(INPUT) //ASSUMPTION: Just a pulse to start it
    val multipleExec = Bool(INPUT)
    val done = Bool(OUTPUT)
  }

  io.dramBaseDst := io.ctrl.dramBaseAddrDst
  io.dramBaseSrc := io.ctrl.dramBaseAddrSrc

  val operands = Vec.fill(myP.nInElemPerWord) { UInt(width = myP.maxInBw)  }


  for (i <- 0 until myP.nInElemPerWord) {
    operands(i) := io.inputStream.bits(myP.maxInBw * (1 + i) - 1, myP.maxInBw * i)
  }


  val serUnit = Module(new SerializerUnit(myP.suparams)).io


  serUnit.counterValue := io.ctrl.actualInBw

  for (i <- 0 until myP.nInElemPerWord) {
    serUnit.input(0)(i) := operands(i)
  }


  // FSM logic for control for starting the SU
  val sSUIdle :: sSUReady :: sSUProcessing :: Nil = Enum(UInt(), 3)
  val regState = Reg(init = UInt(sSUIdle))


  val countToFinish = Reg(init = UInt(0x3F, width = 32 ) )
  val endSerialize = countToFinish ===  UInt(0)

  val filledClscBuff = Reg(init = Bool(false))
  val restartCompSU = serUnit.out.valid & io.outStream.ready

  //end as soon as I have completed a number of columns that is required by the user

  //workaroud for configuration part
  val regStart = Reg(init = Bool(false), next = io.start)

  when(io.start){
    countToFinish := io.ctrl.matrixCols * io.ctrl.matrixRows
    //ASSUMPTION: always padded in nInElemPerWord even if fewer bits as Input
    //while as output padding depends on the number of columns
    //
    //If this is wrong then the count to finish has to change accordingly

  }


  when(serUnit.out.valid) {
    countToFinish := countToFinish - UInt(myP.nInElemPerWord)
  }

  when(regState === sSUIdle) {
    serUnit.start := Bool(false)
    serUnit.out.ready := Bool(false)
    when(io.inputStream.valid & !filledClscBuff) {
      serUnit.start := Bool(true)
      serUnit.out.ready := Bool(true)
      regState := sSUProcessing
    }
  }.elsewhen(regState === sSUReady) {
      io.inputStream.ready := !filledClscBuff
      serUnit.start := Bool(false)
      serUnit.out.ready := Bool(false)

      when(io.inputStream.valid & !endSerialize & !filledClscBuff) {
        //serUnit.start := Bool(true)
        serUnit.out.ready := Bool(true)
        regState := sSUProcessing
      }.elsewhen(filledClscBuff){
        serUnit.out.ready := Bool(false)
        regState := sSUReady
      }.otherwise{
        serUnit.out.ready := Bool(true)
        regState := sSUIdle
      }
    }.elsewhen(regState === sSUProcessing) {
    when(!endSerialize){
      io.inputStream.ready := Bool(false)
      serUnit.start := Bool(true)
      serUnit.out.ready := Bool(false)

      when(restartCompSU) {
        serUnit.out.ready := Bool(true)
        serUnit.start := Bool(true)
        //io.inputStream.ready := Bool(true)
        regState := sSUReady
        }
    }.otherwise{
      serUnit.start := Bool(false)
      regState := sSUIdle
    }
  }.otherwise {
    serUnit.start := Bool(false)
    serUnit.out.ready := Bool(false)
    io.inputStream.ready := Bool(false)

  }
  val valid_r = Reg(init = false.B, next = serUnit.out.valid )
  val valid_pulse =serUnit.out.valid & !valid_r


  //TODO: There is smth that I miss in the configurability part of the component on the matrix size
  val coalescingBuffer = Vec.fill(myP.maxInBw) {
    Vec.fill(myP.outStreamSize) {
      Reg(init = UInt(0, width = 1))
    }
  }
  //val coalescingWire = Vec.fill(myP.nInElemPerWord){UInt()}
  val currentBit = Reg(init = UInt(0, width = myP.maxInBw))
  val currentBuff = Reg(init = UInt(0, width = log2Up(myP.outStreamSize)))

  for (i <- 0 until myP.nInElemPerWord)
    coalescingBuffer(currentBit)(UInt(i) + currentBuff) := coalescingBuffer(currentBit)(UInt(i) + currentBuff)

  //Write enable
  when(!endSerialize | valid_pulse){
    for (i <- 0 until myP.nInElemPerWord)
      coalescingBuffer(currentBit)(UInt(i) + currentBuff) := serUnit.out.bits(0)(i)
  }

  //Reinit the buffer just to new transformation
  val cleanBuff = Bool()
  when(io.start || cleanBuff ){
    for(i <- 0 until myP.maxInBw)
      for (j <- 0 until myP.outStreamSize){
        coalescingBuffer(i)(j) := UInt(0)
      }
  }

  when(serUnit.start) {
    currentBit := currentBit + UInt(1)
  }

  when(io.inputStream.ready && regState === sSUReady) {
    currentBuff := currentBuff + UInt(8)
    currentBit := UInt(0)
  }



  when(io.multipleExec && restartCompSU && currentBuff === UInt(myP.outStreamSize - 8)  && !endSerialize){
    filledClscBuff := Bool(true)
  }



  val clscIndex = Reg(init = UInt(0, width = log2Up(myP.maxInBw) + 1))

  // this can make the difference between configurable or not
  when((endSerialize | filledClscBuff)  && clscIndex < (io.ctrl.actualInBw) ) {
//  when((endSerialize | filledClscBuff)  && clscIndex < UInt(myP.maxInBw) ) {
    cleanBuff := Bool(false)
    io.outStream.valid := Bool(true)
    clscIndex := clscIndex + UInt(1)
    io.done := Bool(true)
  }.elsewhen((endSerialize | filledClscBuff) && clscIndex === io.ctrl.actualInBw ){
    clscIndex := clscIndex
    io.outStream.valid := Bool(false)
    io.done := Bool(false)
    filledClscBuff := Bool(false)
    cleanBuff := Bool(true)
    //anticipate new input stream
    //io.inputStream.ready := Bool(true)
  }.otherwise{
    cleanBuff := Bool(false)
    io.outStream.valid := Bool(false)
    io.done := Bool(false)
    clscIndex := UInt(0)
  }

  io.outStream.bits := coalescingBuffer(clscIndex).asUInt()

/*
  val sumVec = Vec.fill(myP.nInElemPerWord/2){UInt()}

  for(i <- 0 until myP.nInElemPerWord/2 ) {
    sumVec(i) := filteredOps(i*2) + filteredOps(i*2 + 1 )
    // printf("[HW: P2SKrnl] Adding %d + %d\n", filteredOps(i), filteredOps(myP.nInElemPerWord - 1 - i) )

  }

  io.outStream.bits := sumVec.asUInt()
  io.outStream.valid := serUnit.out.valid
*/
  //when(io.outStream.valid){
  //  printf("[HW: P2SKrnl] Input valid \n")
  //}
/*
  when(io.outStream.valid){
      printf("[HW: P2SKrnl] Output valid Data %d\n", io.outStream.bits)

  }
*/

}
