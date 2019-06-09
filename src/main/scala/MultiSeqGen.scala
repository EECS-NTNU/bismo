// Copyright (c) 2018 Norwegian University of Science and Technology (NTNU)
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
import fpgatidbits.synthutils.PrintableParam

// TODO: reusable component, move into tidbits

// MultiSeqGen, "multiple sequence generator", is a sequence generator that
// receives a stream of sequence descriptions (init, count, step) as input.
// each description is a command that generates the corresponding arithmetic
// sequence. when the command is finished, the next command's sequence is
// genertaed.

// example:
// two commands (init=0, count=4, step=1) and (init=10, count=3, step=2)
// would generate the following sequence
// 0 1 2 3 10 12 14

class MultiSeqGenParams(
  val w: Int, val a: Int) extends PrintableParam {
  def headersAsList(): List[String] = {
    return List("datawidth", "countwidth")
  }
  def contentAsList(): List[String] = {
    return List(w, a).map(_.toString)
  }
}

class MultiSeqGenCtrl(p: MultiSeqGenParams) extends Bundle {
  val init = UInt(INPUT, width = p.w)
  val count = UInt(INPUT, width = p.a)
  val step = UInt(INPUT, width = p.w)

  override def cloneType: this.type =
    new MultiSeqGenCtrl(p).asInstanceOf[this.type]
}

class MultiSeqGen(p: MultiSeqGenParams) extends Module {
  val io = new Bundle {
    val in = Decoupled(new MultiSeqGenCtrl(p)).flip
    val out = Decoupled(UInt(width = p.w))
  }
  val regSeqElem = Reg(outType = UInt(width = p.w))
  val regCounter = Reg(outType = UInt(width = p.a))
  val regMaxCount = Reg(outType = UInt(width = p.a))
  val regStep = Reg(outType = UInt(width = p.w))
  io.in.ready := Bool(false)
  io.out.valid := Bool(false)
  io.out.bits := regSeqElem

  val sIdle :: sRun :: Nil = Enum(UInt(), 2)
  val regState = Reg(init = UInt(sIdle))

  /*printf("regState = %d\n", regState)
  printf("regSeqElem = %d\n", regSeqElem)
  printf("regCounter = %d\n", regCounter)
  printf("regMaxCount = %d\n", regMaxCount)
  printf("regStep = %d\n", regStep)*/

  switch(regState) {
    is(sIdle) {
      io.in.ready := Bool(true)
      when(io.in.valid) {
        regState := sRun
        regCounter := UInt(0)
        regSeqElem := io.in.bits.init
        regMaxCount := io.in.bits.count
        regStep := io.in.bits.step
      }
    }

    is(sRun) {
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
}
