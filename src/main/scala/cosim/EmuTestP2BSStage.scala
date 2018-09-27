// Author:  Davide Conficconi
// Date: 27/09/2018
// Revision: 0

package bismo

import Chisel._
import fpgatidbits.dma._
import fpgatidbits.ocm._
import fpgatidbits.streams._
import fpgatidbits.PlatformWrapper._


// standalone, accelerator-wrapped version of Parallel2BSStage
// this provides the hardware side for EmuTestParallel2BSStage
class EmuTestP2BSStage(
                       mRows: Int, mCols: Int,  inBits: Int,  memLatency: Int, countBw: Int, p: PlatformWrapperParams)
  extends GenericAccelerator(p){
  val numMemPorts = 0
  // parameters for accelerator instance
  val myP = new Parallel2BSStageParams(
    suParams = new SerializerUnitParams ( inPrecision = inBits, matrixRows = mRows, matrixCols = mCols,
      staticCounter = true, maxCounterPrec = countBw),
    thMemDepth  = 8, bsMemDepth = 8, inputMemAddr = 0, resMemAddr = 0,
    thMemLatency = memLatency, bramInRegs= memLatency, bramOutRegs = memLatency
  )

  val io = new GenericAcceleratorIF(numMemPorts, p) {
    // base control signals
    val start = Bool(INPUT)                   // hold high while running
    val done = Bool(OUTPUT)                   // high when done until start=0
    val cfg = new Parallel2BSStageCfgIO()
    val ctrl = new Parallel2BSStageCtrlIO(myP).asInput
    // write access to input matrix tile memory
    val inMemory_thr_sel_r = UInt(INPUT, width = log2Up(myP.getRows()))
    val inMemory_thr_sel_c = UInt(INPUT, width = log2Up(myP.getCols()))
    val inMemory_thr_addr = UInt(INPUT, width = log2Up(myP.thMemDepth))
    val inMemory_thr_data = UInt(INPUT, width = myP.getInBits())
    val inMemory_thr_write = Bool(INPUT)

    // access to result memory
    val resmem_addr_r = UInt(INPUT, width = log2Up(myP.getRows()))
    val resmem_addr_e = UInt(INPUT, width = log2Up(myP.resMemAddr))
    val resmem_data = UInt(OUTPUT, width = myP.getCols())
  }


  val rmm = Module(new Parallel2BSStage(myP)).io
  rmm.cfg <> io.cfg
  rmm.ctrl <> io.ctrl
  rmm.start := io.start
  io.done := rmm.done
  // the signature can be e.g. used for checking that the accelerator has the
  // correct version. here the signature is regenerated from the current date.
  io.signature := makeDefaultSignature()

  val inMemory_thr = Vec.fill(myP.getRows()) { Vec.fill(myP.getCols()){
    Module(new PipelinedDualPortBRAM(
      addrBits = log2Up(myP.thMemDepth), dataBits = myP.getInBits(),
      regIn = myP.bramInRegs, regOut = myP.bramOutRegs
    )).io
  }}


  val resmem = Vec.fill(myP.getRows()) { Module(new PipelinedDualPortBRAM(
      addrBits = log2Up(myP.resMemAddr), dataBits = myP.getCols(),
      regIn = 0, regOut = 0
    )).io
  }


  for(i <- 0 until myP.getRows())
    for(j <- 0 until myP.getCols()){
      inMemory_thr(i)(j).ports(0).req.addr := io.inMemory_thr_addr
      inMemory_thr(i)(j).ports(0).req.writeData := io.inMemory_thr_data
      val myWriteEn = (io.inMemory_thr_sel_r === UInt(i) && io.inMemory_thr_sel_c === UInt(j)) & io.inMemory_thr_write
      inMemory_thr(i)(j).ports(0).req.writeEn := myWriteEn
      /*when(io.start){
      printf("[CO-HW] Input Data %d with write enable %d and address %d %d\n", io.inMemory_thr_data, io.inMemory_thr_write, io.inMemory_thr_sel_r ,io.inMemory_thr_sel_c)
      }*/
      inMemory_thr(i)(j).ports(1).req := rmm.inMemory.thr_req(i)(j)
      rmm.inMemory.thr_rsp(i)(j) := inMemory_thr(i)(j).ports(1).rsp
    }

  // wire-up: result memory
  for(i <- 0 until myP.getRows()) {
      resmem(i).ports(0).req <> rmm.res.req(i)
      resmem(i).ports(1).req.addr := io.resmem_addr_e
      resmem(i).ports(1).req.writeEn := Bool(false)
    }

  // register result reg selector inputs
  io.resmem_data := resmem(io.resmem_addr_r).ports(1).rsp.readData
}
