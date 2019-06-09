// Copyright (c) 2019 Xilinx

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
import fpgatidbits.streams._
import fpgatidbits.hlstools.TemplatedHLSBlackBox

class ExecAddrGenParams(
  val addrUnit: Int,
  val outAddrBitwidth: Int,
  val constAddr: Int
)

class ExecAddrGenOutput extends PrintableBundle {
  val writeAddr = UInt(width = 1)
  val writeEn = UInt(width = 1)
  val shiftAmount = UInt(width = 5)
  val negate = UInt(width = 1)
  val clear = UInt(width = 1)
  val last = UInt(width = 1)
  val rhsAddr = UInt(width = 16)
  val lhsAddr = UInt(width = 16)

  override def cloneType: this.type =
    new ExecAddrGenOutput().asInstanceOf[this.type]

  val printfStr = "(lhs = %d, rhs = %d, shift %d, wen? %d, waddr? %d last? %d clear? %d neg? %d)\n"
  val printfElems = { () ⇒ Seq(lhsAddr, rhsAddr, shiftAmount, writeEn, writeAddr, last, clear, negate) }
}

class ExecAddrGen(val p: ExecAddrGenParams) extends TemplatedHLSBlackBox {
  val io = new Bundle {
    val in = Decoupled(UInt(width = BISMOLimits.descrBits)).flip
    val out = Decoupled(UInt(width = BISMOLimits.execAddrGenOutBits))
    val rst_n = Bool(INPUT)
    in.bits.setName("in_V_V_TDATA")
    in.valid.setName("in_V_V_TVALID")
    in.ready.setName("in_V_V_TREADY")
    out.bits.setName("out_V_V_TDATA")
    out.valid.setName("out_V_V_TVALID")
    out.ready.setName("out_V_V_TREADY")
    rst_n.setName("ap_rst_n")
  }
  // clock needs to be added manually to BlackBox
	addClock(Driver.implicitClock)
  renameClock("clk", "ap_clk")

  val hlsTemplateParams: Map[String, String] = Map(
    "ADDR_UNIT" -> p.addrUnit.toString,
    "OUT_ADDR_BITWIDTH" -> p.outAddrBitwidth.toString,
    "CONSTANT_ADDRESS" -> p.constAddr.toString
  )
}
