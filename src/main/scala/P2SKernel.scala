
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


class P2SKernel (myP: P2SKernelParams) extends Module {
  val io = new Bundle {
    val actualPrecision = UInt(INPUT, width = myP.maxInBw)
    val inputStream = Decoupled(UInt(width = myP.maxInBw * myP.nInElemPerWord)).flip()
    val outStream = Decoupled(UInt(width = myP.mrp.dataWidth))
  }

//Input stream filter
  val operands = Vec.fill(myP.nInElemPerWord) { UInt(width = myP.maxInBw)  }
  for (i <- 0 until myP.nInElemPerWord) {
    operands(i) := io.inputStream.bits(myP.maxInBw * (1 + i) - 1, myP.maxInBw * i)
  }

  val inReg = Vec.fill(myP.nInElemPerWord) { Reg(init = UInt(0,width = myP.maxInBw))  }
  when(io.inputStream.ready & io.inputStream.valid){
    for (i <- 0 until myP.nInElemPerWord) {
      inReg(i) := operands(i)
    }
  }

//SU init
  val serUnit = Module(new SerializerUnit(myP.suparams)).io
  serUnit.counterValue := io.actualPrecision
  serUnit.start := Bool(false)
  serUnit.out.ready := Bool(false)

  for (i <- 0 until myP.nInElemPerWord) {
    serUnit.input(0)(i) := inReg(i)
  }
//Coalescing Buffer init
  val coalescingBuffer = Vec.fill(myP.maxInBw) {
    Vec.fill(myP.outStreamSize) {
      Reg(init = UInt(0, width = 1))
    }
  }

  val currentWriteBit = Reg(init = UInt(0, width = myP.maxInBw))
  val currentWriteBuff = Reg(init = UInt(0, width = log2Up(myP.outStreamSize) + 1))
  val clscIndex = Reg(init = UInt(0, width = log2Up(myP.maxInBw) + 1))
  val filledClscBuff = Reg(init = Bool(false))
  val writeEnable = Reg(init = Bool(false))
  val cleanBuff = Reg(init = Bool(false))



  // FSM logic for SU computation
  val sSUIdle :: sSUReady :: sSUProcessing :: sEmptyCoalescing :: sClean :: Nil = Enum(UInt(), 5)
  val regState = Reg(init = UInt(sSUIdle))

  //Default values
  io.inputStream.ready := Bool(false)
  serUnit.out.ready := Bool(false)
  io.outStream.valid := Bool(false)
  serUnit.start := Bool(false)




  switch(regState){

    is (sSUIdle){
      io.inputStream.ready := Bool(true)
      serUnit.out.ready := Bool(true)
      cleanBuff := Bool(false)
      when(io.inputStream.valid){
        writeEnable := Bool(true)
        regState := sSUProcessing
      }

    }
    is (sSUReady){
      //clean
      writeEnable := Bool(false)
      cleanBuff := Bool(false)
      serUnit.out.ready := Bool(true)
      when(currentWriteBuff === UInt(myP.outStreamSize - 8)){//filled clsc buff
        regState := sEmptyCoalescing
      }.elsewhen(io.inputStream.valid){ // go on processing
        currentWriteBuff := currentWriteBuff + UInt(8)
        currentWriteBit := UInt(0)
        io.inputStream.ready := Bool(true)
        writeEnable := Bool(true)
        regState := sSUProcessing
      }
    }
    is (sSUProcessing){
      io.inputStream.ready := Bool(false)
      serUnit.out.ready := Bool(false)
      serUnit.start := Bool(true)
      when(currentWriteBit < io.actualPrecision){
        currentWriteBit := currentWriteBit + UInt(1)
        writeEnable := Bool(true)

      }.otherwise{
        writeEnable := Bool(false)
      }
      when(serUnit.out.valid){
        regState := sSUReady
        currentWriteBit := currentWriteBit
      }
    }
    is (sEmptyCoalescing) {
      io.outStream.valid := Bool(true)
      when(io.outStream.ready){
        clscIndex := clscIndex + UInt(1)
        when(clscIndex === io.actualPrecision - UInt(1)){
          regState := sClean
        }
      }

    }
    is (sClean){
      clscIndex := UInt(0)
      currentWriteBit := UInt(0)
      currentWriteBuff := UInt(0)
      cleanBuff := Bool(true)
      regState := sSUIdle

    }

  }


//  when(writeEnable){
//    currentWriteBit := currentWriteBit + UInt(1)
//  }

  /*

  when(serUnit.start) {
    currentWriteBit := currentWriteBit + UInt(1)
  }

  when(io.inputStream.ready && regState === sSUReady) {
    currentWriteBuff := currentWriteBuff + UInt(8)
    currentWriteBit := UInt(0)
  }*/


  //Write-clean coalescing buff
  when(writeEnable){
    for (i <- 0 until myP.nInElemPerWord)
      coalescingBuffer(currentWriteBit)(UInt(i) + currentWriteBuff) := serUnit.out.bits(0)(i)

  }.elsewhen(cleanBuff){

    for(i <- 0 until myP.maxInBw)
      for (j <- 0 until myP.outStreamSize){
        coalescingBuffer(i)(j) := UInt(0)
      }
  }.otherwise{
    for (i <- 0 until myP.nInElemPerWord)
      coalescingBuffer(currentWriteBit)(UInt(i) + currentWriteBuff) := coalescingBuffer(currentWriteBit)(UInt(i) + currentWriteBuff)
  }
/*
  for (i <- 0 until myP.nInElemPerWord)
    coalescingBuffer(currentWriteBit)(UInt(i) + currentWriteBuff) := coalescingBuffer(currentWriteBit)(UInt(i) + currentWriteBuff)

  //Write enable
  when(!endSerialize | valid_pulse){
    for (i <- 0 until myP.nInElemPerWord)
      coalescingBuffer(currentWriteBit)(UInt(i) + currentWriteBuff) := serUnit.out.bits(0)(i)
  }

  when(io.start || cleanBuff ){
    for(i <- 0 until myP.maxInBw)
      for (j <- 0 until myP.outStreamSize){
        coalescingBuffer(i)(j) := UInt(0)
      }
  }
*/
/*

  val countToFinish = Reg(init = UInt(0x3F, width = 32 ) )
  val endSerialize = countToFinish ===  UInt(0)


  val restartCompSU = serUnit.out.valid & io.outStream.ready



  //end as soon as I have completed a number of streams that is required by the user


  when(io.start){
    countToFinish := io.ctrl.matrixCols * io.ctrl.matrixRows
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


  //TODO: There is smth that I miss in the configurability part of the component on the matrix size?

  //Reinit the buffer just to new transformation
  val cleanBuff = Bool()

  for (i <- 0 until myP.nInElemPerWord)
    coalescingBuffer(currentWriteBit)(UInt(i) + currentWriteBuff) := coalescingBuffer(currentWriteBit)(UInt(i) + currentWriteBuff)

  val valid_pulse_delayed = ShiftRegister(valid_pulse, 1)
  //Write enable
  when(!endSerialize | valid_pulse){
    for (i <- 0 until myP.nInElemPerWord)
      coalescingBuffer(currentWriteBit)(UInt(i) + currentWriteBuff) := serUnit.out.bits(0)(i)
  }


  when(io.start || cleanBuff ){
    for(i <- 0 until myP.maxInBw)
      for (j <- 0 until myP.outStreamSize){
        coalescingBuffer(i)(j) := UInt(0)
      }
  }

  when(serUnit.start) {
    currentWriteBit := currentWriteBit + UInt(1)
  }

  when(io.inputStream.ready && regState === sSUReady) {
    currentWriteBuff := currentWriteBuff + UInt(8)
    currentWriteBit := UInt(0)
  }



  when(io.multipleExec && restartCompSU && currentWriteBuff === UInt(myP.outStreamSize - 8)  && !endSerialize){
    filledClscBuff := Bool(true)
  }



  // this can make the difference between configurable or not in output the stream
  when((endSerialize | filledClscBuff)  && clscIndex < (io.ctrl.actualPrecision) ) {
//  when((endSerialize | filledClscBuff)  && clscIndex < UInt(myP.maxInBw) ) {
    cleanBuff := Bool(false)
    io.outStream.valid := Bool(true)
    clscIndex := clscIndex + UInt(1)
    io.done := Bool(true)
  }.elsewhen((endSerialize | filledClscBuff) && clscIndex === io.ctrl.actualPrecision ){
    clscIndex := clscIndex
    io.outStream.valid := Bool(false)
    io.done := Bool(false)
    filledClscBuff := Bool(false)
    cleanBuff := io.multipleExec
  }.otherwise{
    cleanBuff := Bool(false)
    io.outStream.valid := Bool(false)
    io.done := Bool(false)
    clscIndex := UInt(0)
  }*/


  io.outStream.bits := coalescingBuffer(clscIndex).asUInt()


}
