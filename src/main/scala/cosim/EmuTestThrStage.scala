
// Author:  Davide Conficconi
// Date: 13/09/2018
// Revision: 0

package bismo

import Chisel._
import fpgatidbits.dma._
import fpgatidbits.ocm._
import fpgatidbits.streams._
import fpgatidbits.PlatformWrapper._

// standalone, accelerator-wrapped version of ThrStage
// this provides the hardware side for EmuTestThrStage
class EmuTestThrStage(
  mRows: Int, mCols: Int, inBits: Int, outBits: Int, thUnroll: Int, p: PlatformWrapperParams)
  extends GenericAccelerator(p) {
  val numMemPorts = 0
  // parameters for accelerator instance
  val myP = new ThrStageParams(
    thresholdMemDepth = 8, inputMemDepth = 8, resMemDepth = 8, bramInRegs = 1, bramOutRegs = 1,
    thuParams = new ThresholdingUnitParams(
      thBBParams = new ThresholdingBuildingBlockParams(inPrecision = inBits, popcountUnroll = thUnroll, outPrecision = outBits),
      inputBitPrecision = inBits, maxOutputBitPrecision = outBits, matrixRows = mRows,
      matrixColumns = mCols, unrollingFactorOutputPrecision = thUnroll,
      unrollingFactorRows = mRows, unrollingFactorColumns = mCols))

  val io = new GenericAcceleratorIF(numMemPorts, p) {
    // base control signals
    val start = Bool(INPUT) // hold high while running
    val done = Bool(OUTPUT) // high when done until start=0
    val cfg = new ThrStageCfgIO()
    val ctrl = new ThrStageCtrlIO(myP).asInput
    // write access to input matrix tile memory
    val inMemory_thr_sel_r = UInt(INPUT, width = log2Up(myP.getUnrollRows()))
    val inMemory_thr_sel_c = UInt(INPUT, width = log2Up(myP.getThUnroll()))
    val inMemory_thr_addr = UInt(INPUT, width = log2Up(myP.thresholdMemDepth))
    val inMemory_thr_data = UInt(INPUT, width = myP.getInBits())
    val inMemory_thr_write = Bool(INPUT)
    val inMemory_act_sel_r = UInt(INPUT, width = log2Up(myP.getUnrollRows()))
    val inMemory_act_sel_c = UInt(INPUT, width = log2Up(myP.getUnrollCols()))
    val inMemory_act_addr = UInt(INPUT, width = log2Up(myP.inputMemDepth))
    val inMemory_act_data = UInt(INPUT, width = myP.getInBits())
    val inMemory_act_write = Bool(INPUT)
    // access to result memory
    val resmem_addr_r = UInt(INPUT, width = log2Up(myP.getUnrollRows()))
    val resmem_addr_c = UInt(INPUT, width = log2Up(myP.getUnrollCols()))
    val resmem_addr_e = UInt(INPUT, width = log2Up(myP.resMemDepth))
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

  val inMemory_thr = Vec.fill(myP.getUnrollRows()) {
    Vec.fill(myP.getThUnroll()) {
      Module(new PipelinedDualPortBRAM(
        addrBits = log2Up(myP.thresholdMemDepth), dataBits = myP.getInBits(),
        regIn = myP.bramInRegs, regOut = myP.bramOutRegs)).io
    }
  }
  val inMemory_act = Vec.fill(myP.getUnrollRows()) {
    Vec.fill(myP.getUnrollCols()) {
      Module(new PipelinedDualPortBRAM(
        addrBits = log2Up(myP.inputMemDepth), dataBits = myP.getInBits(),
        regIn = myP.bramInRegs, regOut = myP.bramOutRegs)).io
    }
  }

  val resmem = Vec.fill(myP.getUnrollRows()) {
    Vec.fill(myP.getUnrollCols()) {
      Module(new PipelinedDualPortBRAM(
        addrBits = log2Up(myP.resMemDepth), dataBits = myP.getResBitWidth(),
        regIn = 0, regOut = 0)).io
    }
  }

  for (i <- 0 until myP.getUnrollRows())
    for (j <- 0 until myP.getThUnroll()) {
      inMemory_thr(i)(j).ports(0).req.addr := io.inMemory_thr_addr
      inMemory_thr(i)(j).ports(0).req.writeData := io.inMemory_thr_data
      val myWriteEn = (io.inMemory_thr_sel_r === UInt(i) && io.inMemory_thr_sel_c === UInt(j)) & io.inMemory_thr_write
      inMemory_thr(i)(j).ports(0).req.writeEn := myWriteEn
      /*when(io.start){
      printf("[CO-HW] Input Thr %d with write enable %d and address %d %d\n", io.inMemory_thr_data, io.inMemory_thr_write, io.inMemory_thr_sel_r ,io.inMemory_thr_sel_c)
      }*/
      inMemory_thr(i)(j).ports(1).req := rmm.inMemory.thr_req(i)(j)
      rmm.inMemory.thr_rsp(i)(j) := inMemory_thr(i)(j).ports(1).rsp
    }
  for (i <- 0 until myP.getUnrollRows())
    for (j <- 0 until myP.getUnrollCols()) {
      inMemory_act(i)(j).ports(0).req.addr := io.inMemory_act_addr
      inMemory_act(i)(j).ports(0).req.writeData := io.inMemory_act_data
      val myWriteEn = (io.inMemory_act_sel_r === UInt(i) && io.inMemory_act_sel_c === UInt(j)) & io.inMemory_act_write
      inMemory_act(i)(j).ports(0).req.writeEn := myWriteEn
      /*when(io.start){
      printf("[CO-HW] Input Activation %d, write enable %d and address %d %d\n", io.inMemory_act_data, io.inMemory_act_write, io.inMemory_act_sel_r,io.inMemory_act_sel_c)
      }*/
      inMemory_act(i)(j).ports(1).req := rmm.inMemory.act_req(i)(j)
      rmm.inMemory.act_rsp(i)(j) := inMemory_act(i)(j).ports(1).rsp
    }
  // wire-up: result memory
  for (i <- 0 until myP.getUnrollRows())
    for (j <- 0 until myP.getUnrollCols()) {
      resmem(i)(j).ports(0).req <> rmm.res.req(i)(j)
      resmem(i)(j).ports(1).req.addr := io.resmem_addr_e
      resmem(i)(j).ports(1).req.writeEn := Bool(false)
    }

  // register result reg selector inputs
  io.resmem_data := resmem(io.resmem_addr_r)(io.resmem_addr_c).ports(1).rsp.readData
}
