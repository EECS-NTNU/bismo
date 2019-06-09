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
import fpgatidbits.profiler._

// The DecoupledController receives instructions, sets up parameters for its
// stage, launches and monitors stage execution, and performs token enq/deq
// operations to ensure the stages execute in correct order.
// Each stage has its own derived *Controller class, defined at the bottom of
// this file.

// base class for all stage controllers, taking in a stream of commands and
// individually executing each while respecting shared resource access locks.
// support having multiple outstanding dispatched instructions until a sync
// instruction is encountered

class DecoupledController[Ts <: Bundle, Ti <: Bundle](
  inChannels: Int,          // number of input sync channels
  outChannels: Int,         // number of output sync channels
  genStageO: => Ts,         // gentype for stage output
  genInstr: => Ti,          // gentype for instructions
  instr2StageO: Ti => Ts    // convert instruction to stage output
) extends Module {
  val io = new Bundle {
    // instruction queue input
    val op = Decoupled(genInstr).flip
    // enable/disable execution of new instructions for this stage
    val enable = Bool(INPUT)
    // run instructions to stage
    val stage_run = Decoupled(genStageO)
    // completed run instructions from stage
    val stage_done = Valid(Bool()).flip
    // synchronization channels
    val sync_in = Vec.fill(inChannels) { Decoupled(Bool()).flip }
    val sync_out = Vec.fill(outChannels) { Decoupled(Bool()) }
    // state profiler output
    val perf = new Bundle {
      val start = Bool(INPUT)
      val count = UInt(OUTPUT, 32)
      val sel = UInt(INPUT, log2Up(4))
    }
  }
  val instrAsSync = new BISMOSyncInstruction().fromBits(io.op.bits.toBits())
  // default values
  io.op.ready := Bool(false)
  io.stage_run.bits := instr2StageO(io.op.bits)
  io.stage_run.valid := Bool(false)
  for(i <- 0 until inChannels) { io.sync_in(i).ready := Bool(false) }
  for(i <- 0 until outChannels) {
    io.sync_out(i).valid := Bool(false)
    io.sync_out(i).bits := Bool(false)
  }
  // keep track of outstanding running instructions
  val regOutstandingRunCmds = Reg(init = UInt(0, width = 32))
  when(io.stage_run.fire() && !io.stage_done.fire()) {
    regOutstandingRunCmds := regOutstandingRunCmds + UInt(1)
  } .elsewhen(!io.stage_run.fire() && io.stage_done.fire()) {
    regOutstandingRunCmds := regOutstandingRunCmds - UInt(1)
  }

  val sDispatch :: sWaitComplete :: sSend :: sReceive :: Nil = Enum(UInt(), 4)
  val regState = Reg(init = UInt(sDispatch))

  // NOTE: the following finite state machine assumes that valid will not go
  // once it has gone high. to ensure this, add a FIFO queue to feed the op
  // and runcfg inputs of the controller.
  // TODO maybe get rid of wait states for higher performance
  switch(regState) {
    is(sDispatch) {
      when(io.op.valid & io.enable) {
        when(instrAsSync.isRunCfg) {
          // runcfg instruction, dispatch
          io.stage_run.valid := Bool(true)
          when(io.stage_run.ready) {
            // fetch next instruction
            regState := sDispatch
            // pop from command queue when done
            io.op.ready := Bool(true)
          }
        } .elsewhen(!instrAsSync.isRunCfg) {
          // sync instruction, wait for run completion
          regState := sWaitComplete
        }
      }
    }
    is(sWaitComplete) {
      // sync instructions require all runcfgs to be complete
      when(regOutstandingRunCmds === UInt(0)) {
        regState := Mux(instrAsSync.isSendToken, sSend, sReceive)
      }
    }
    is(sSend) {
      // send sync token
      val sendChannel = io.sync_out(instrAsSync.chanID)
      sendChannel.valid := Bool(true)
      when(sendChannel.ready) {
        regState := sDispatch
        io.op.ready := Bool(true)
      }
    }
    is(sReceive) {
      // receive sync token
      val receiveChannel = io.sync_in(instrAsSync.chanID)
      receiveChannel.ready := Bool(true)
      when(receiveChannel.valid) {
        regState := sDispatch
        io.op.ready := Bool(true)
      }
    }
  }
  // define an easier-to-interpret "profile state":
  // 0 - no commands to execute, idle
  // 1 - running
  // 2 - send token
  // 3 - receive token
  val regProfileState = Reg(init = UInt(0, width = log2Up(4)))
  when(regOutstandingRunCmds > UInt(0)) {
    regProfileState := UInt(1)
  } .elsewhen(io.op.valid === Bool(false)) {
    regProfileState := UInt(0)
  } .otherwise {
    regProfileState := regState
  }
  // state profiler
  val profiler = Module(new StateProfiler(4)).io
  profiler <> io.perf
  profiler.start := io.perf.start & io.enable
  profiler.probe := regProfileState
}

// derived classes for each type of controller.
class FetchDecoupledController extends DecoupledController(
  genStageO = new FetchStageCtrlIO(), inChannels = 1, outChannels = 1,
  genInstr = new BISMOFetchRunInstruction(),
  instr2StageO = (x: BISMOFetchRunInstruction) => x.runcfg
){
  /*val prevState = Reg(next=regState)
  when(regState != prevState) {
    printf("FetchController state: %d -> %d\n", prevState, regState)
  }*/
}

class ExecDecoupledController extends DecoupledController(
  genStageO = new ExecStageCtrlIO(), inChannels = 2, outChannels = 2,
  genInstr = new BISMOExecRunInstruction(),
  instr2StageO = (x: BISMOExecRunInstruction) => x.runcfg
){
  /*val prevState = Reg(next=regState)
  when(regState != prevState) {
    printf("ExecController state: %d -> %d\n", prevState, regState)
  }*/
}

class ResultDecoupledController extends DecoupledController(
  genStageO = new ResultStageCtrlIO(), inChannels = 1, outChannels = 1,
  genInstr = new BISMOResultRunInstruction(),
  instr2StageO = (x: BISMOResultRunInstruction) => x.runcfg
){
  /*val prevState = Reg(next=regState)
  when(regState != prevState) {
    printf("ResultController state: %d -> %d\n", prevState, regState)
  }*/
}
