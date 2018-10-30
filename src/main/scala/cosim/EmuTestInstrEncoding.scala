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
import fpgatidbits.PlatformWrapper._


// test to ensure that hardware and software encode/decode instrs in the
// same way. this is accomplished by letting software write to a 128-bit
// register, whose contents are re-interpreted and made available field by
// field on the hardware side. the software can then do a loopback and compare
// what it gets out to ensure it was the same as the input.

class EmuTestInstrEncoding(p: PlatformWrapperParams) extends GenericAccelerator(p) {
  val numMemPorts = 0
  val io = new GenericAcceleratorIF(numMemPorts, p) {
    val raw_instr_in = UInt(INPUT, width = BISMOLimits.instrBits)
    val sync_instr_out = new BISMOSyncInstruction().asOutput()
    val fr_instr_out = new BISMOFetchRunInstruction().asOutput()
    val er_instr_out = new BISMOExecRunInstruction().asOutput()
    val rr_instr_out = new BISMOResultRunInstruction().asOutput()
  }
  io.signature := makeDefaultSignature()

  // static check to ensure all instr bitwidths are as defined in BISMOLimits
  Predef.assert(io.sync_instr_out.getWidth() == BISMOLimits.instrBits)
  Predef.assert(io.fr_instr_out.getWidth() == BISMOLimits.instrBits)
  Predef.assert(io.er_instr_out.getWidth() == BISMOLimits.instrBits)
  Predef.assert(io.rr_instr_out.getWidth() == BISMOLimits.instrBits)

  // reinterpret raw input as different instructions and give as output
  io.sync_instr_out := io.sync_instr_out.fromBits((io.raw_instr_in))
  io.fr_instr_out := io.fr_instr_out.fromBits((io.raw_instr_in))
  io.er_instr_out := io.er_instr_out.fromBits((io.raw_instr_in))
  io.rr_instr_out := io.rr_instr_out.fromBits((io.raw_instr_in))
}
