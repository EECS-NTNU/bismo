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
import fpgatidbits.PlatformWrapper._

// standalone, accelerator-wrapped version of ExecStage
// this provides the hardware side for EmuTestExecStage
class EmuTestExecStage(p: PlatformWrapperParams) extends GenericAccelerator(p) {
  val numMemPorts = 0
  // parameters for accelerator instance
  val myP = new ExecStageParams(
    dpaParams = new DotProductArrayParams(
      dpuParams = new DotProductUnitParams(
        pcParams = new PopCountUnitParams(numInputBits = 64),
        accWidth = 32, maxShiftSteps = 16
      ), m = 2, n = 2
    ), lhsTileMem = 1024, rhsTileMem = 1024, tileMemAddrUnit = 1
  )
  val io = new GenericAcceleratorIF(numMemPorts, p) {
    // base control signals
    val start = Bool(INPUT) // hold high while running
    val done = Bool(OUTPUT) // high when done until start=0
    val cfg = new ExecStageCfgIO()
    val csr = new ExecStageCtrlIO(myP).asInput
    // write access to input matrix tile memory
    val tilemem_lhs_sel = UInt(INPUT, width = log2Up(myP.getM()))
    val tilemem_lhs_addr = UInt(INPUT, width = log2Up(myP.lhsTileMem))
    val tilemem_lhs_data = UInt(INPUT, width = myP.getK())
    val tilemem_lhs_write = Bool(INPUT)
    val tilemem_rhs_sel = UInt(INPUT, width = log2Up(myP.getN()))
    val tilemem_rhs_addr = UInt(INPUT, width = log2Up(myP.rhsTileMem))
    val tilemem_rhs_data = UInt(INPUT, width = myP.getK())
    val tilemem_rhs_write = Bool(INPUT)
    // access to result memory
    val resmem_addr_r = UInt(INPUT, width = log2Up(myP.getM()))
    val resmem_addr_c = UInt(INPUT, width = log2Up(myP.getN()))
    val resmem_addr_e = UInt(INPUT, width = log2Up(myP.resEntriesPerMem))
    val resmem_data = UInt(OUTPUT, width = myP.getResBitWidth())
  }
  val rmm = Module(new ExecStage(myP)).io
  rmm.cfg <> io.cfg
  rmm.csr <> io.csr
  rmm.start := io.start
  io.done := rmm.done
  // the signature can be e.g. used for checking that the accelerator has the
  // correct version. here the signature is regenerated from the current date.
  io.signature := makeDefaultSignature()

  // tile memories
  val tilemem_lhs = Vec.fill(myP.getM()) {
    Module(new PipelinedDualPortBRAM(
      addrBits = log2Up(myP.lhsTileMem), dataBits = myP.getK(),
      regIn = myP.bramInRegs, regOut = myP.bramOutRegs
    )).io
  }
  val tilemem_rhs = Vec.fill(myP.getN()) {
    Module(new PipelinedDualPortBRAM(
      addrBits = log2Up(myP.rhsTileMem), dataBits = myP.getK(),
      regIn = myP.bramInRegs, regOut = myP.bramOutRegs
    )).io
  }
  // instantiate the result memory
  val resmem = Vec.fill(myP.getM()) {
    Vec.fill(myP.getN()) {
      Module(new PipelinedDualPortBRAM(
        addrBits = log2Up(myP.resEntriesPerMem), dataBits = myP.getResBitWidth(),
        regIn = 0, regOut = 0
      )).io
    }
  }
  // wire up direct access to tile mems
  // use port 0 for direct muxed access, port 1 for actually feeding the DPA
  for (i ← 0 until myP.getM()) {
    tilemem_lhs(i).ports(0).req.addr := io.tilemem_lhs_addr
    tilemem_lhs(i).ports(0).req.writeData := io.tilemem_lhs_data
    val myWriteEn = (io.tilemem_lhs_sel === UInt(i)) & io.tilemem_lhs_write
    tilemem_lhs(i).ports(0).req.writeEn := myWriteEn

    tilemem_lhs(i).ports(1).req := rmm.tilemem.lhs_req(i)
    rmm.tilemem.lhs_rsp(i) := tilemem_lhs(i).ports(1).rsp
    //when(tilemem_lhs(i).ports(0).req.writeEn) { printf("BRAM %d write: addr %d data %x\n", UInt(i), tilemem_lhs(i).ports(0).req.addr, tilemem_lhs(i).ports(0).req.writeData) }
  }
  for (i ← 0 until myP.getN()) {
    tilemem_rhs(i).ports(0).req.addr := io.tilemem_rhs_addr
    tilemem_rhs(i).ports(0).req.writeData := io.tilemem_rhs_data
    val myWriteEn = (io.tilemem_rhs_sel === UInt(i)) & io.tilemem_rhs_write
    tilemem_rhs(i).ports(0).req.writeEn := myWriteEn

    tilemem_rhs(i).ports(1).req := rmm.tilemem.rhs_req(i)
    rmm.tilemem.rhs_rsp(i) := tilemem_rhs(i).ports(1).rsp
  }
  // wire-up: result memory
  for {
    m ← 0 until myP.getM()
    n ← 0 until myP.getN()
  } {
    resmem(m)(n).ports(0).req := rmm.res.req(m)(n)
    resmem(m)(n).ports(1).req.addr := io.resmem_addr_e
    resmem(m)(n).ports(1).req.writeEn := Bool(false)
    // resultStage.resmem_rsp(m)(n) := resmem(m)(n).ports(1).rsp
  }

  // register result reg selector inputs
  // resmem(io.resmem_addr_r)(io.resmem_addr_c).ports(1).req.addr :=
  io.resmem_data := resmem(io.resmem_addr_r)(io.resmem_addr_c).ports(1).rsp.readData

}
