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
import fpgatidbits.dma._
import fpgatidbits.ocm._
import fpgatidbits.streams._
import fpgatidbits.PlatformWrapper._

class EmuTestFetchStage(
  nLHS: Int, nRHS: Int, p: PlatformWrapperParams
) extends GenericAccelerator(p) {
  val numMemPorts = 1
  val myP = new FetchStageParams(
    numLHSMems = nLHS, numRHSMems = nRHS,
    numAddrBits = 10, mrp = PYNQZ1Params.toMemReqParams(), bramWrLat = 2
  )
  val io = new GenericAcceleratorIF(numMemPorts, p) {
    // base control signals
    val start = Bool(INPUT)                   // hold high while running
    val done = Bool(OUTPUT)                   // high when done until start=0
    val perf = new FetchStagePerfIO(myP)
    val csr = new FetchStageCtrlIO().asInput
    val bram_sel = UInt(INPUT, width = 32)
    val bram_req = new OCMRequest(myP.mrp.dataWidth, myP.numAddrBits).asInput
    val bram_rsp = UInt(OUTPUT, width = myP.mrp.dataWidth)
  }
  io.signature := makeDefaultSignature()

  val brams = Vec.fill(myP.numNodes) {
    Module(new PipelinedDualPortBRAM(
      addrBits = myP.numAddrBits, dataBits = myP.mrp.dataWidth,
      regIn = 1, regOut = 1
    )).io
  }

  val fetch = Module(new FetchStage(myP)).io
  fetch.start := io.start
  io.done := fetch.done
  fetch.csr <> io.csr
  fetch.perf <> io.perf
  for(i <- 0 until myP.numLHSMems) {
    brams(i).ports(0).req <> io.bram_req
    brams(i).ports(0).req.writeEn := (io.bram_sel === UInt(i)) & io.bram_req.writeEn
    brams(i).ports(1).req <> fetch.bram.lhs_req(i)
  }
  for(i <- 0 until myP.numRHSMems) {
    val o = i + myP.numLHSMems
    brams(o).ports(0).req <> io.bram_req
    brams(o).ports(0).req.writeEn := (io.bram_sel === UInt(o)) & io.bram_req.writeEn
    brams(o).ports(1).req <> fetch.bram.rhs_req(i)
  }
  io.bram_rsp := brams(io.bram_sel).ports(0).rsp.readData

  fetch.dram.rd_req <> io.memPort(0).memRdReq
  io.memPort(0).memRdRsp <> fetch.dram.rd_rsp

  // write ports are unused -- plug them to prevent Vivado synth errors
  for(i <- 0 until numMemPorts) {
    plugMemWritePort(i)
  }
}
