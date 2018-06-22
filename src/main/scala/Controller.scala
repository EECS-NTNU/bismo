// Copyright (c) 2018 Norwegian University of Science and Technology (NTNU)
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
import fpgatidbits.profiler._

// The Controller Module receives instructions, sets up parameters for its
// stage, launches and monitors stage execution, and performs token enq/deq
// operations to ensure the stages execute in correct order.
// Each stage has its own derived *Controller class, defined at the bottom of
// this file.

// Currently, there are two queues associated with each controller:
// - an "op" queue that only contains an opcode (run, send token, receive token)
//   and the token queue ID to send/receive from if applicable
// - a separate "runcfg" queue that contains stage-specific configuration. for
//   each run opcode, an item will be popped from this queue.
object Opcodes {
  val opRun :: opSendToken :: opReceiveToken :: Nil = Enum(UInt(), 3)
}

class ControllerCmd(inChannels: Int, outChannels: Int) extends Bundle {
  val opcode = UInt(width = 2)
  val token_channel = UInt(width = log2Up(math.max(inChannels, outChannels)))

  override def cloneType: this.type =
    new ControllerCmd(inChannels, outChannels).asInstanceOf[this.type]
}

// base class for all stage controllers, taking in a stream of commands and
// individually executing each while respecting shared resource access locks.
class BaseController[Ts <: Bundle](
  inChannels: Int,          // number of input sync channels
  outChannels: Int,         // number of output sync channels
  genStageO: => Ts          // gentype for stage output
) extends Module {
  val io = new Bundle {
    // command queue input
    val op = Decoupled(new ControllerCmd(inChannels, outChannels)).flip
    // config for run commands
    val runcfg = Decoupled(genStageO).flip
    // enable/disable execution of new instructions for this stage
    val enable = Bool(INPUT)
    // stage start/stop signals to stage
    val start = Bool(OUTPUT)
    val done = Bool(INPUT)
    // output to stage (config for current run)
    val stageO = genStageO.asOutput
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
  // default values
  io.op.ready := Bool(false)
  io.runcfg.ready := Bool(false)
  io.start := Bool(false)
  io.stageO := io.runcfg.bits
  for(i <- 0 until inChannels) { io.sync_in(i).ready := Bool(false) }
  for(i <- 0 until outChannels) {
    io.sync_out(i).valid := Bool(false)
    io.sync_out(i).bits := Bool(false)
  }

  val sGetCmd :: sRun :: sSend :: sReceive :: Nil = Enum(UInt(), 4)
  val regState = Reg(init = UInt(sGetCmd))

  // NOTE: the following finite state machine assumes that valid will not go
  // once it has gone high. to ensure this, add a FIFO queue to feed the op
  // and runcfg inputs of the controller.
  // TODO maybe get rid of wait states for higher performance
  switch(regState) {
    is(sGetCmd) {
      when(io.op.valid & io.enable) {
        // "peek" into the new command:
        when(io.op.bits.opcode === Opcodes.opRun && io.runcfg.valid && !io.done) {
          regState := sRun
        } .elsewhen(io.op.bits.opcode === Opcodes.opSendToken) {
          regState := sSend
        } .elsewhen(io.op.bits.opcode === Opcodes.opReceiveToken) {
          regState := sReceive
        }
      }
    }
    is(sRun) {
      // run stage
      io.start := Bool(true)
      when(io.done) {
        // pop from command queue when done
        io.op.ready := Bool(true)
        io.runcfg.ready := Bool(true)
        // get new command
        regState := sGetCmd
      }
    }
    is(sSend) {
      // send sync token
      val sendChannel = io.sync_out(io.op.bits.token_channel)
      sendChannel.valid := Bool(true)
      when(sendChannel.ready) {
        regState := sGetCmd
        io.op.ready := Bool(true)
      }
    }
    is(sReceive) {
      // receive sync token
      val receiveChannel = io.sync_in(io.op.bits.token_channel)
      receiveChannel.ready := Bool(true)
      when(receiveChannel.valid) {
        regState := sGetCmd
        io.op.ready := Bool(true)
      }
    }
  }

  // state profiler
  val profiler = Module(new StateProfiler(4)).io
  profiler <> io.perf
  profiler.start := io.perf.start & io.enable
  profiler.probe := regState
}

// derived classes for each type of controller.
class FetchController(val myP: FetchStageParams) extends BaseController(
  genStageO = new FetchStageCtrlIO(myP), inChannels = 1, outChannels = 1
){ }

class ExecController(val myP: ExecStageParams) extends BaseController(
  genStageO = new ExecStageCtrlIO(myP), inChannels = 2, outChannels = 2
){ }

class ResultController(val myP: ResultStageParams) extends BaseController(
  genStageO = new ResultStageCtrlIO(myP), inChannels = 1, outChannels = 1
){ }
