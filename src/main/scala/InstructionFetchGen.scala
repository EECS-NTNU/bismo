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

class InstructionFetchGen extends Module {
  val io = new Bundle {
    val start = Bool(INPUT)
    // fetch instructions
    val in = Decoupled(new BufDesc()).flip
    // total number of instructions (finish condition)
    val total = UInt(INPUT, width = 32)
    // current number of instructions in stage queue
    val queue_count = UInt(INPUT, width = 16)
    // when a new fetch will be triggered
    val queue_threshold = UInt(INPUT, width = 16)
    // monitor incoming instructions to limit request rate
    // connect this to .fire() for the stage instr queue (non-instr-fetch)
    val new_instr_pulse = Bool(INPUT)
    // generated fetch instructions
    val out = Decoupled(new BISMOFetchRunInstruction())
  }
  val regReceivedInstrs = Reg(init = UInt(0, width = 32))
  val regBytesLeftNextSeg = Reg(init = UInt(0, width = 32))
  val runcfg = new FetchStageCtrlIO()
  io.out.bits.isRunCfg := Bool(true)  // always runcfg
  io.out.bits.targetStage := UInt(0)  // always target fetch stage
  runcfg.tiles_per_row := UInt(0)
  runcfg.dram_block_count := UInt(1)
  runcfg.dram_block_offset_bytes := UInt(0)
  runcfg.dram_block_size_bytes := io.in.bits.bytes
  runcfg.dram_base := io.in.bits.ptr
  io.out.bits.runcfg := runcfg

  io.out.valid := Bool(false)
  io.in.ready := Bool(false)

  val sIdle :: sMonitorLevels :: sDoFetch :: sCountResponses :: sFinished :: Nil = Enum(UInt(), 5)
  val regState = Reg(init = UInt(sIdle))
  switch(regState) {
    is(sIdle) {
      when(io.start) {
        regReceivedInstrs := UInt(0)
        regState := sMonitorLevels
      }
    }

    is(sMonitorLevels) {
      when(regReceivedInstrs === io.total) {
        regState := sFinished
      } .otherwise {
        // monitor instruction levels in FIFO. if it goes below threshold,
        // issue an instruction fetch.
        when(io.queue_count < io.queue_threshold) {
          regState := sDoFetch
        }
      }
    }

    is(sDoFetch) {
      // allow one instruction fetch to go through
      io.out.valid := io.in.valid
      io.in.ready := io.out.ready
      when(io.out.fire()) {
        regState := sCountResponses
        regBytesLeftNextSeg := UInt(io.in.bits.bytes)
      }
    }

    is(sCountResponses) {
      // wait for issued request to be fully responded to
      // when we have finished all instructions, finish
      when(io.new_instr_pulse) {
        regBytesLeftNextSeg := regBytesLeftNextSeg - UInt(BISMOLimits.instrBits/8)
        regReceivedInstrs := regReceivedInstrs + UInt(1)
        when(regBytesLeftNextSeg === UInt(BISMOLimits.instrBits/8)) {
          regState := sMonitorLevels
        }
      }
    }

    is(sFinished) {
      //io.finished := Bool(true)
      when(!io.start) {
        regState := sIdle
      }
    }
  }

}
