
// Author:  Davide Conficconi
// Date: 13/09/2018
// Revision: 0

package bismo

import Chisel._
import fpgatidbits.ocm._
import fpgatidbits.streams._
import fpgatidbits.PlatformWrapper._


// standalone, accelerator-wrapped version of ThrStage
// this provides the hardware side for EmuTestThrStage
class EmuTestThrStage (/*Other needed params*/p: PlatformWrapperParams) extends GenericAccelerator(p){
  val numMemPorts = 0
  // parameters for accelerator instance
  val myP = new ThrStageParams(
      thresholdMemDepth = 8, inputMemAddr = 8, resMemAddr = 8, bramInRegs = 1, bramOutRegs = 1,
      thuParams = new ThresholdingUnitParams(
        thBBParams = new ThresholdingBuildingBlockParams(	inPrecision = 32, popcountUnroll = 1,  outPrecision = 2),
        inputBitPrecision = 32, maxOutputBitPrecision = 2, matrixRows = 2,
        matrixColumns = 2, thresholdMemDepth = 8,  unrollingFactorOutputPrecision = 1,
        unrollingFactorRows = 2, unrollingFactorColumns = 2
      )
  )
  //TODO I'm here :)
  val io = new GenericAcceleratorIF(numMemPorts, p) {
    // base control signals
    val start = Bool(INPUT)                   // hold high while running
    val done = Bool(OUTPUT)                   // high when done until start=0
    val cfg = new ThrStageCfgIO()
    val ctrl = new ThrStageCtrlIO(myP).asInput
    // write access to input matrix tile memory
    val inMemory_thr_sel = UInt(INPUT, width = log2Up(myP.getUnrollRows()))
    val inMemory_thr_addr = UInt(INPUT, width = log2Up(myP.thresholdMemDepth))
    val inMemory_thr_data = UInt(INPUT, width = myP.getInBits())
    val inMemory_thr_write = Bool(INPUT)
    val inMemory_act_sel = UInt(INPUT, width = log2Up(myP.getUnrollRows()))
    val inMemory_act_addr = UInt(INPUT, width = log2Up(myP.inputMemAddr))
    val inMemory_act_data = UInt(INPUT, width = myP.getInBits())
    val inMemory_act_write = Bool(INPUT)
    // access to result memory
    val resmem_addr_r = UInt(INPUT, width = log2Up(myP.getUnrollRows()))
    val resmem_addr_e = UInt(INPUT, width = log2Up(myP.resMemAddr))
    val resmem_data = UInt(OUTPUT, width = myP.getResBitWidth())
  }
  val rmm = Module(new ThrStage(myP)).io
  rmm.cfg <> io.cfg
  rmm.ctrl <> io.ctrl
  rmm.start := io.start
  io.done := rmm.done
  // the signature can be e.g. used for checking that the accelerator has the
  // correct version. here the signature is regenerated from the current date.
  io.signature := makeDefaultSignature()

  // tile memories
  val inMemory_thr = Vec.fill(myP.getUnrollRows()) {
    Module(new PipelinedDualPortBRAM(
      addrBits = log2Up(myP.thresholdMemDepth), dataBits = myP.getInBits(),
      regIn = myP.bramInRegs, regOut = myP.bramOutRegs
    )).io
  }
  val inMemory_act = Vec.fill(myP.getUnrollRows()) {
    Module(new PipelinedDualPortBRAM(
      addrBits = log2Up(myP.inputMemAddr), dataBits = myP.getInBits(),
      regIn = myP.bramInRegs, regOut = myP.bramOutRegs
    )).io
  }
  // instantiate the result memory
  val resmem = Vec.fill(myP.getUnrollRows()) {
    Module(new PipelinedDualPortBRAM(
      addrBits = log2Up(myP.resMemAddr), dataBits = myP.getResBitWidth(),
      regIn = 0, regOut = 0
    )).io
  }
  // wire up direct access to tile mems
  // use port 0 for direct muxed access, port 1 for actually feeding the DPA
  for(i <- 0 until myP.getUnrollRows()) {
    inMemory_thr(i).ports(0).req.addr := io.inMemory_thr_addr
    inMemory_thr(i).ports(0).req.writeData := io.inMemory_thr_data
    val myWriteEn = (io.inMemory_thr_sel === UInt(i)) & io.inMemory_thr_write
    inMemory_thr(i).ports(0).req.writeEn := myWriteEn

    inMemory_thr(i).ports(1).req := rmm.inMemory.thr_req(i)
    rmm.inMemory.thr_rsp(i) := inMemory_thr(i).ports(1).rsp
    //when(inMemory_thr(i).ports(0).req.writeEn) { printf("BRAM %d write: addr %d data %x\n", UInt(i), inMemory_thr(i).ports(0).req.addr, inMemory_thr(i).ports(0).req.writeData) }
  }
  for(i <- 0 until myP.getUnrollRows()) {
    inMemory_act(i).ports(0).req.addr := io.inMemory_act_addr
    inMemory_act(i).ports(0).req.writeData := io.inMemory_act_data
    val myWriteEn = (io.inMemory_act_sel === UInt(i)) & io.inMemory_act_write
    inMemory_act(i).ports(0).req.writeEn := myWriteEn

    inMemory_act(i).ports(1).req := rmm.inMemory.act_req(i)
    rmm.inMemory.act_rsp(i) := inMemory_act(i).ports(1).rsp
  }
  // wire-up: result memory
  for{
    m <- 0 until myP.getUnrollRows()
  } {
    resmem(m).ports(0).req := rmm.res.req(m)//(n)
    resmem(m).ports(1).req.addr := io.resmem_addr_e
    resmem(m).ports(1).req.writeEn := Bool(false)
    // resultStage.resmem_rsp(m)(n) := resmem(m)(n).ports(1).rsp
  }

  // register result reg selector inputs
  // resmem(io.resmem_addr_r)(io.resmem_addr_c).ports(1).req.addr :=
  io.resmem_data := resmem(io.resmem_addr_r).ports(1).rsp.readData

}
