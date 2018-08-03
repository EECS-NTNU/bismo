// Copyright (c) 2018 Xilinx
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
// * Neither the name of [project] nor the names of its
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

class BufDesc extends Bundle {
  val ptr = UInt(width = 32)
  val bytes = UInt(width = 32)

  override def cloneType: this.type =
    new BufDesc().asInstanceOf[this.type]
}

class InstructionFetchGen(ifgName: String = "") extends Module {
  val io = new Bundle {
    val enable = Bool(INPUT)
    // input instruction fetch instructions
    val in = Decoupled(new BISMOFetchRunInstruction()).flip
    // current number of instructions in stage queue
    val queue_count = UInt(INPUT, width = 16)
    // when a new fetch will be triggered
    val queue_threshold = UInt(INPUT, width = 16)
    // monitor incoming instructions to limit request rate
    val new_instr_pulse = Bool(INPUT)
    // output instructon fetch instructions
    val out = Decoupled(new BISMOFetchRunInstruction())
  }
  // count number of outstanding instruction fetches
  val regOutstandingInstrsToFetch = Reg(init = UInt(0, width = 16))
  when(io.out.fire()) {
    val samt = log2Up(BISMOLimits.instrBits / 8)
    regOutstandingInstrsToFetch := io.in.bits.runcfg.dram_block_size_bytes >> samt
  } .otherwise {
    when(io.new_instr_pulse) {
      regOutstandingInstrsToFetch := regOutstandingInstrsToFetch - UInt(1)
      when(regOutstandingInstrsToFetch === UInt(0)) {
        printf(s"ERROR: InstructionFetchGen $ifgName underflow!\n")
      }
    }
  }
  // only issue a new instruction fetch when:
  // - there is enough room in the instruction queue
  // - there are no current instruction fetches outstanding
  val under_threshold = io.queue_count < io.queue_threshold
  val can_issue = (regOutstandingInstrsToFetch === UInt(0))
  val allow = (under_threshold & can_issue & io.enable)
  StreamThrottle(io.in, !allow) <> io.out
  // uncomment to debug
  /*val prevOutstanding = Reg(next=regOutstandingInstrsToFetch)
  when(prevOutstanding != regOutstandingInstrsToFetch) {
    printf(s"$ifgName outstanding=%d->%d\n", prevOutstanding, regOutstandingInstrsToFetch)
  }*/
  /*val couldIssue = Reg(next=can_issue)
  when(couldIssue != can_issue) {
    printf(s"$ifgName can issue %d->%d\n", couldIssue, can_issue)
  }*/
}
