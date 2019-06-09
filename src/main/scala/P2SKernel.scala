// Copyright (c) 2019 Xilinx
//
// BSD v3 License
//
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// * Redistributions of source code must retain the above copyright notice, this
//   list of conditions and the following disclaimer.
//
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
//
// * Neither the name of BISMO nor the names of its
//   contributors may be used to endorse or promote products derived from
//   this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

package bismo

import Chisel._
import fpgatidbits.ocm._
import fpgatidbits.streams._
import fpgatidbits.dma._
import fpgatidbits.math.Counter
import fpgatidbits.synthutils.PrintableParam

class P2SKernelParams(
  val maxInBw: Int = 8,
  val nInElemPerWord: Int = 8,
  val outStreamSize: Int = 64,
  val mrp: MemReqParams) extends PrintableParam {

  def headersAsList(): List[String] = {
    return List("M", "N", "O")
  }
  def contentAsList(): List[String] = {
    return List(maxInBw, nInElemPerWord, outStreamSize).map(_.toString)
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

  for (i <- 0 until myP.nInElemPerWord) {
    // copy bits from input stream when valid and ready
    shifters(i).parWrEn := io.inputStream.fire()
    shifters(i).parIn := io.inputStream.bits((i + 1) * myP.maxInBw - 1, i * myP.maxInBw)
    shifters(i).shiftEn := doBitShift
    // serial input is unused
    shifters(i).serIn := UInt(0)
  }
  val currentBitData = Cat(shifters.map(_.serOut).reverse)
  // write buffer for coalescing
  val writeBuffers = Vec.fill(myP.maxInBw) {
    Module(new SerialInParallelOut(parWidth = myP.outStreamSize, serWidth = myP.nInElemPerWord)).io
  }
  for (i <- 0 until myP.maxInBw) {
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
        }.otherwise {
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

class P2SKernel_Fast(myP: P2SKernelParams) extends Module {
  val io = new Bundle {
    // actual precision of the input bit-parallel matrix, <= maxInBw
    // this field must be able to represent maxInBw, hence the +1
    val actualPrecision = UInt(INPUT, width = log2Up(myP.maxInBw) + 1)
    val inputStream = Decoupled(UInt(width = myP.maxInBw * myP.nInElemPerWord)).flip()
    val outStream = Decoupled(UInt(width = myP.mrp.dataWidth))
  }
  val currentBit = Module(new Counter(log2Up(myP.maxInBw) + 1)).io
  currentBit.nsteps := io.actualPrecision
  currentBit.enable := Bool(false)

  val currentGroup = Module(new Counter(log2Up(myP.maxInBw) + 1)).io
  currentGroup.nsteps := UInt(myP.outStreamSize / myP.nInElemPerWord)
  currentGroup.enable := Bool(false)

  val bitpar_elems = Vec.tabulate(myP.nInElemPerWord) {
    i: Int => io.inputStream.bits((i + 1) * myP.maxInBw - 1, i * myP.maxInBw)
  }

  // write buffers for coalescing
  val writeBuffers = Vec.fill(myP.maxInBw) {
    Module(new SerialInParallelOut(parWidth = myP.outStreamSize, serWidth = myP.nInElemPerWord)).io
  }

  for (i <- 0 until myP.maxInBw) {
    writeBuffers(i).shiftEn := io.inputStream.fire()
    val currentBitData = Cat(bitpar_elems.map(_(i)).reverse)
    writeBuffers(i).serIn := currentBitData
  }

  val currentWriteBuffer = writeBuffers(currentBit.current)

  io.inputStream.ready := Bool(false)
  io.outStream.valid := Bool(false)

  io.outStream.bits := currentWriteBuffer.parOut

  val sRead :: sWrite :: Nil = Enum(UInt(), 2)
  val regState = Reg(init = UInt(sRead))

  switch(regState) {
    is(sRead) {
      io.inputStream.ready := Bool(true)
      when(io.inputStream.valid) {
        currentGroup.enable := Bool(true)
        when(currentGroup.full) {
          regState := sWrite
        }.otherwise {
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
