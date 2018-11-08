
// Author:  Davide Conficconi
// Date: 17/10/2018
// Revision: 0

package bismo

import Chisel._
import fpgatidbits.ocm._
import fpgatidbits.streams._
import fpgatidbits.dma._
import fpgatidbits.math.Counter

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

class P2SKernel_Slow(myP: P2SKernelParams) extends Module {
  val io = new Bundle {
    // actual precision of the input bit-parallel matrix, <= maxInBw
    // this field must be able to represent maxInBw, hence the +1
    val actualPrecision = UInt(INPUT, width = log2Up(myP.maxInBw) + 1)
    val inputStream = Decoupled(UInt(width = myP.maxInBw * myP.nInElemPerWord)).flip()
    val outStream = Decoupled(UInt(width = myP.mrp.dataWidth))
  }
  val doBitShift = Bool()
  doBitShift := Bool(false)
  val currentBit = Module(new Counter(log2Up(myP.maxInBw) + 1)).io
  currentBit.nsteps := io.actualPrecision
  currentBit.enable := doBitShift

  val currentGroup = Module(new Counter(log2Up(myP.maxInBw) + 1)).io
  currentGroup.nsteps := UInt(myP.outStreamSize / myP.nInElemPerWord)
  currentGroup.enable := Bool(false)

  val shifters = Vec.fill(myP.nInElemPerWord) {
    Module(new ParallelInSerialOut(parWidth = myP.maxInBw, serWidth = 1)).io
  }

  for(i <- 0 until myP.nInElemPerWord) {
    // copy bits from input stream when valid and ready
    shifters(i).parWrEn := io.inputStream.fire()
    shifters(i).parIn := io.inputStream.bits((i+1) * myP.maxInBw - 1, i * myP.maxInBw)
    shifters(i).shiftEn := doBitShift
    // serial input is unused
    shifters(i).serIn := UInt(0)
  }
  val currentBitData = Cat(shifters.map(_.serOut).reverse)
  // write buffer for coalescing
  val writeBuffers = Vec.fill(myP.maxInBw) {
    Module(new SerialInParallelOut(parWidth = myP.outStreamSize, serWidth = myP.nInElemPerWord)).io
  }
  for(i <- 0 until myP.maxInBw) {
    writeBuffers(i).shiftEn := Bool(false)
    writeBuffers(i).serIn := currentBitData
  }
  val currentWriteBuffer = writeBuffers(currentBit.current)

  io.inputStream.ready := Bool(false)
  io.outStream.valid := Bool(false)

  io.outStream.bits := currentWriteBuffer.parOut

  val sRead :: sShift :: sWrite :: Nil = Enum(UInt(), 3)
  val regState = Reg(init = UInt(sRead))

  switch(regState) {
    is(sRead) {
      io.inputStream.ready := Bool(true)
      when(io.inputStream.valid) {
        regState := sShift
      }
    }

    is(sShift) {
      doBitShift := Bool(true)
      currentWriteBuffer.shiftEn := Bool(true)
      when(currentBit.full) {
        currentGroup.enable := Bool(true)
        when(currentGroup.full) {
          regState := sWrite
        } .otherwise {
          regState := sRead
        }
      }
    }

    is(sWrite) {
      io.outStream.valid := Bool(true)
      when(io.outStream.ready) {
        currentBit.enable := Bool(true)
        when(currentBit.full) {
          regState := sRead
        }
      }
    }
  }
}

class P2SKernel(myP: P2SKernelParams) extends Module {
  val io = new Bundle {
    // actual precision of the input bit-parallel matrix, <= maxInBw
    // this field must be able to represent maxInBw, hence the +1
    val actualPrecision = UInt(INPUT, width = log2Up(myP.maxInBw) + 1)
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
