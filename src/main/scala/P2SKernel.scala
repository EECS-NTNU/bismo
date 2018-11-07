
// Author:  Davide Conficconi
// Date: 17/10/2018
// Revision: 0

package bismo

import Chisel._
import fpgatidbits.ocm._
import fpgatidbits.streams._
import fpgatidbits.dma._

class P2SKernelParams(
    val maxInBw: Int = 8,
    val nInElemPerWord: Int = 8,
    val outStreamSize: Int = 64,
    val mrp: MemReqParams,
    val suparams: SerializerUnitParams) extends PrintableParam {

  //The rolling should be less or equal then the maximum input bit-width and a divisor of it
  val precRolling = suparams.unrollingFactor
  //if static counter the unroll should be equal to 1
  Predef.assert((suparams.staticCounter && precRolling == 1) || !suparams.staticCounter)
  Predef.assert(precRolling <= maxInBw)
  Predef.assert(maxInBw % precRolling == 0)

  val writingUnit: Int = precRolling

  def headersAsList(): List[String] = {
    return List("M", "N", "O", "R")
  }
  def contentAsList(): List[String] = {
    return List(maxInBw, nInElemPerWord, outStreamSize, precRolling).map(_.toString)
  }
}

class P2SKernel(myP: P2SKernelParams) extends Module {
  val io = new Bundle {
    val actualPrecision = UInt(INPUT, width = myP.maxInBw)
    val inputStream = Decoupled(UInt(width = myP.maxInBw * myP.nInElemPerWord)).flip()
    val outStream = Decoupled(UInt(width = myP.mrp.dataWidth))
  }

  //Input stream filter
  val operands = Vec.fill(myP.nInElemPerWord) { UInt(width = myP.maxInBw) }
  for (i ← 0 until myP.nInElemPerWord) {
    operands(i) := io.inputStream.bits(myP.maxInBw * (1 + i) - 1, myP.maxInBw * i)
  }

  val inReg = Vec.fill(myP.nInElemPerWord) { Reg(init = UInt(0, width = myP.maxInBw)) }
  when(io.inputStream.ready & io.inputStream.valid) {
    for (i ← 0 until myP.nInElemPerWord) {
      inReg(i) := operands(i)
    }
  }

  //SU init
  val serUnit = Module(new SerializerUnit(myP.suparams)).io
  serUnit.counterValue := io.actualPrecision
  serUnit.start := Bool(false)
  serUnit.out.ready := Bool(false)

  for (i ← 0 until myP.nInElemPerWord) {
    serUnit.input(0)(i) := inReg(i)
  }
  //Coalescing Buffer init
  val coalescingBuffer = Vec.fill(myP.maxInBw) {
    Vec.fill(myP.outStreamSize) {
      Reg(init = UInt(0, width = 1))
    }
  }

  val currentWriteBit = Reg(init = UInt(0, width = myP.maxInBw + 1))
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

  switch(regState) {

    is(sSUIdle) {
      io.inputStream.ready := Bool(true)
      serUnit.out.ready := Bool(true)
      cleanBuff := Bool(false)
      when(io.inputStream.valid) {
        writeEnable := Bool(true)
        regState := sSUProcessing
      }

    }
    is(sSUReady) {
      //clean
      writeEnable := Bool(false)
      cleanBuff := Bool(false)
      serUnit.out.ready := Bool(true)
      when(currentWriteBuff === UInt(myP.outStreamSize - myP.nInElemPerWord)) { //filled clsc buff
        regState := sEmptyCoalescing
      }.elsewhen(io.inputStream.valid) { // go on processing
        currentWriteBuff := currentWriteBuff + UInt(myP.nInElemPerWord)
        currentWriteBit := UInt(0)
        io.inputStream.ready := Bool(true)
        writeEnable := Bool(true)
        regState := sSUProcessing
      }
    }
    is(sSUProcessing) {
      io.inputStream.ready := Bool(false)
      serUnit.out.ready := Bool(false)
      serUnit.start := Bool(true)
      //      when(currentWriteBit + UInt(myP.writingUnit) < io.actualPrecision){

      when(currentWriteBit < io.actualPrecision) {
        //        currentWriteBit := currentWriteBit + UInt(1)
        currentWriteBit := currentWriteBit + UInt(myP.writingUnit)
        writeEnable := Bool(true)

      }.otherwise {
        writeEnable := Bool(false)
      }
      when(serUnit.out.valid) {
        regState := sSUReady
        currentWriteBit := currentWriteBit
      }
    }
    is(sEmptyCoalescing) {
      io.outStream.valid := Bool(true)
      when(io.outStream.ready) {
        clscIndex := clscIndex + UInt(1)
        when(clscIndex === io.actualPrecision - UInt(1)) {
          regState := sClean
        }
      }

    }
    is(sClean) {
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
  when(writeEnable) {
    for (j ← 0 until myP.writingUnit)
      for (i ← 0 until myP.nInElemPerWord) {
        //        coalescingBuffer(UInt(j))(UInt(i) + currentWriteBuff) := serUnit.out.bits(0)(i)(j,j)
        coalescingBuffer(currentWriteBit + UInt(j))(UInt(i) + currentWriteBuff) := serUnit.out.bits(0)(i)(j)
      }
  }.elsewhen(cleanBuff) {

    for (i ← 0 until myP.maxInBw)
      for (j ← 0 until myP.outStreamSize) {
        coalescingBuffer(i)(j) := UInt(0)
      }
  }.otherwise {

    for (i ← 0 until myP.maxInBw)
      for (j ← 0 until myP.outStreamSize) {
        coalescingBuffer(i)(j) := coalescingBuffer(i)(j)
      }
    //    for (i <- 0 until myP.nInElemPerWord)
    //      for(j <- 0 until myP.writingUnit)
    //        coalescingBuffer(currentWriteBit + UInt(j) )(UInt(i) + currentWriteBuff) := coalescingBuffer(currentWriteBit + UInt(j) )(UInt(i) + currentWriteBuff)
  }

  io.outStream.bits := coalescingBuffer(clscIndex).asUInt()

}
