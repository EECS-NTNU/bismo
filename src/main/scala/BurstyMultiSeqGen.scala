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

// TODO: reusable component, move into tidbits

// "bursting" version of MultiSeqGen. a burst size is specified at synthesis
// time in terms of a power-of-two. the generator first takes large burst-sized
// steps if possible, then finishes the sequence with unit steps afterwards.

class BurstyMultiSeqGenParams(
  w: Int, a: Int, val burstShift: Int
) extends MultiSeqGenParams(w, a) {
  override def headersAsList(): List[String] = {
    return List("datawidth", "countwidth", "burstshift")
  }
  override def contentAsList(): List[String] = {
    return List(w, a, burstShift).map(_.toString)
  }
}

class BurstyMultiSeqGenOutput(p: BurstyMultiSeqGenParams) extends Bundle {
  val ind = UInt(width = p.w)
  val step = UInt(width = p.w)
  override def cloneType: this.type =
    new BurstyMultiSeqGenOutput(p).asInstanceOf[this.type]
}

class BurstyMultiSeqGen(p: BurstyMultiSeqGenParams) extends Module {
  val io = new Bundle {
    val in = Decoupled(new MultiSeqGenCtrl(p)).flip
    val out = Decoupled(new BurstyMultiSeqGenOutput(p))
  }
  val regSeqElem = Reg(outType = UInt(width = p.w))
  val regCounter = Reg(outType = UInt(width = p.a))
  val regMaxCount = Reg(outType = UInt(width = p.a))
  val regMaxCountWithBurst = Reg(outType = UInt(width = p.a))
  val regStep = Reg(outType = UInt(width = p.w))
  io.in.ready := Bool(false)
  io.out.valid := Bool(false)
  io.out.bits.ind := regSeqElem
  io.out.bits.step := regStep

  val sIdle :: sBurst :: sRun :: Nil = Enum(UInt(), 3)
  val regState = Reg(init = UInt(sIdle))

  switch(regState) {
    is(sIdle) {
      io.in.ready := Bool(true)
      when(io.in.valid) {
        regState := sBurst
        regCounter := UInt(0)
        regSeqElem := io.in.bits.init
        regMaxCount := io.in.bits.count
        // calculate the max count we can reach with bursts
        regMaxCountWithBurst := Cat((io.in.bits.count >> p.burstShift), UInt(0, width=p.burstShift))
        // start by using burst step size
        regStep := io.in.bits.step << p.burstShift
      }
    }

    is(sBurst) {
      // produce burst-sized steps
      when(regCounter === regMaxCountWithBurst) {
        regState := sRun
        // switch back to unit step size
        regStep := regStep >> p.burstShift
      }.otherwise {
        io.out.valid := Bool(true)
        when(io.out.ready) {
          regCounter := regCounter + (UInt(1) << p.burstShift)
          regSeqElem := regSeqElem + regStep
        }
      }
    }

    is(sRun) {
      // produce unit-sized steps
      when(regCounter === regMaxCount) {
        regState := sIdle
      }.otherwise {
        io.out.valid := Bool(true)
        when(io.out.ready) {
          regCounter := regCounter + UInt(1)
          regSeqElem := regSeqElem + regStep
        }
      }
    }
  }

  /*printf("regState = %d\n", regState)
  printf("regSeqElem = %d\n", regSeqElem)
  printf("regCounter = %d\n", regCounter)
  printf("regMaxCount = %d\n", regMaxCount)
  printf("regMaxCountWithBurst = %d\n", regMaxCountWithBurst)
  printf("regStep = %d\n", regStep)*/
}
