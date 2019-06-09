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
import fpgatidbits.streams.PrintableBundle

class BISMOInstruction extends PrintableBundle {
  val instrData = UInt(width = 128 - 3)
  val isRunCfg = Bool()
  val targetStage = UInt(width = 2)

  val printfStr = "instruction: targetStage = %d isRuncfg = %d \n"
  val printfElems = { () ⇒ Seq(targetStage, isRunCfg) }

  override def cloneType: this.type =
    new BISMOInstruction().asInstanceOf[this.type]
}

class BISMOSyncInstruction extends PrintableBundle {
  val unused = UInt(width = 128 - 6)
  // channel number for token sync
  val chanID = UInt(width = 2)
  // send token if true, receive if false
  val isSendToken = Bool()
  // always false since this is a sync instruction
  val isRunCfg = Bool()
  // which stage this instruction is targeting (TargetStages)
  val targetStage = UInt(width = 2)

  val printfStr = "sync: targetStage = %d isSend = %d chanID = %d \n"
  val printfElems = { () ⇒ Seq(targetStage, isSendToken, chanID) }

  override def cloneType: this.type =
    new BISMOSyncInstruction().asInstanceOf[this.type]
}

class BISMOFetchRunInstruction extends PrintableBundle {
  val runcfg = new FetchStageCtrlIO()
  val unused = UInt(width = 3)
  // always true
  val isRunCfg = Bool()
  // always stgFetch
  val targetStage = UInt(width = 2)

  val printfStr = "(targetStage = %d isRuncfg = %d) runcfg: " + runcfg.printfStr
  val printfElems = {() => Seq(targetStage, isRunCfg) ++ runcfg.printfElems()}

  override def cloneType: this.type =
    new BISMOFetchRunInstruction().asInstanceOf[this.type]
}

class BISMOExecRunInstruction extends PrintableBundle {
  val runcfg = new ExecStageCtrlIO()
  val unused = UInt(width = 68)
  // always true
  val isRunCfg = Bool()
  // always stgExec
  val targetStage = UInt(width = 2)

  val printfStr = "(targetStage = %d isRuncfg = %d) runcfg: " + runcfg.printfStr
  val printfElems = {() => Seq(targetStage, isRunCfg) ++ runcfg.printfElems()}

  override def cloneType: this.type =
    new BISMOExecRunInstruction().asInstanceOf[this.type]
}

class BISMOResultRunInstruction extends PrintableBundle {
  val runcfg = new ResultStageCtrlIO()
  val unused = UInt(width = 59)
  // always true
  val isRunCfg = Bool()
  // always stgResult
  val targetStage = UInt(width = 2)

  val printfStr = "(targetStage = %d isRuncfg = %d) runcfg: " + runcfg.printfStr
  val printfElems = {() => Seq(targetStage, isRunCfg) ++ runcfg.printfElems()}

  override def cloneType: this.type =
    new BISMOResultRunInstruction().asInstanceOf[this.type]
}
