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
import fpgatidbits.dma._
import fpgatidbits.ocm._
import fpgatidbits.streams._
import fpgatidbits.PlatformWrapper._

class EmuTestResultStage(
  accArrayDim: Int, p: PlatformWrapperParams) extends GenericAccelerator(p) {
  val numMemPorts = 1
  // parameters for accelerator instance
  val myP = new ResultStageParams(
    accWidth = 32, resMemReadLatency = 0,
    dpa_rhs = accArrayDim, dpa_lhs = accArrayDim, mrp = PYNQZ1Params.toMemReqParams())
  val io = new GenericAcceleratorIF(numMemPorts, p) {
    // base control signals
    val start = Bool(INPUT)                   // hold high while running
    val done = Bool(OUTPUT)                   // high when done until start=0
    val csr = new ResultStageCtrlIO().asInput
    val accwr_en = Bool(INPUT)
    val accwr_lhs = UInt(INPUT, width = log2Up(myP.dpa_lhs))
    val accwr_rhs = UInt(INPUT, width = log2Up(myP.dpa_rhs))
    val accwr_data = UInt(INPUT, width = myP.accWidth)
  }
  val resmem = Vec.fill(myP.dpa_lhs) {
    Vec.fill(myP.dpa_rhs) {
      Module(new PipelinedDualPortBRAM(
        addrBits = 1, dataBits = myP.accWidth, regIn = 0, regOut = 0)).io
    }
  }
  val res = Module(new ResultStage(myP)).io
  res.start := io.start
  io.done := res.done
  res.csr <> io.csr
  for (lhs ← 0 until myP.dpa_lhs) {
    for (rhs ← 0 until myP.dpa_rhs) {
      // drive defaults on resmem req port 0
      val is_my_lhs = (UInt(lhs) === io.accwr_lhs)
      val is_my_rhs = (UInt(rhs) === io.accwr_rhs)
      val is_my_write = is_my_lhs & is_my_rhs
      // write enable selected resmem entry
      resmem(lhs)(rhs).ports(0).req.writeEn := is_my_write & io.accwr_en
      resmem(lhs)(rhs).ports(0).req.addr := UInt(0)
      resmem(lhs)(rhs).ports(0).req.writeData := io.accwr_data
      // connect resmem port 1 directly to ResultStage interface
      res.resmem_req(lhs)(rhs) <> resmem(lhs)(rhs).ports(1).req
      resmem(lhs)(rhs).ports(1).rsp <> res.resmem_rsp(lhs)(rhs)
    }
  }

  // connect DRAM interface for ResultStage
  res.dram.wr_req <> io.memPort(0).memWrReq
  res.dram.wr_dat <> io.memPort(0).memWrDat
  io.memPort(0).memWrRsp <> res.dram.wr_rsp
  // plug unused read port
  plugMemReadPort(0)
  // the signature can be e.g. used for checking that the accelerator has the
  // correct version. here the signature is regenerated from the current date.
  io.signature := makeDefaultSignature()
}
