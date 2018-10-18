package bismo

import Chisel._
import fpgatidbits.ocm._
import fpgatidbits.dma._
import fpgatidbits.streams._
import fpgatidbits.PlatformWrapper._



class EmuTestP2SAccel(
                          accArrayDim: Int, p: PlatformWrapperParams
                        ) extends GenericAccelerator(p) {
  val numMemPorts = 1
  // parameters for accelerator instance
  val myP = new StandAloneP2SParams( maxInBw = 8, nInElemPerWord = 8, outStreamSize = 8,
   mrp = PYNQZ1Params.toMemReqParams()
  )

  val io = new GenericAcceleratorIF(numMemPorts, p) {
    // base control signals
    val start = Bool(INPUT)                   // hold high while running
    val done = Bool(OUTPUT)                   // high when done until start=0
    val csr = new P2SKernelCtrlIO(myP.p2sparams).asInput
    val accwr_en = Bool(INPUT)
//    val accwr_lhs = UInt(INPUT, width = log2Up(myP.dpa_lhs))
//    val accwr_rhs = UInt(INPUT, width = log2Up(myP.dpa_rhs))
//    val accwr_data = UInt(INPUT, width = myP.accWidth)
  }
//  val resmem = Vec.fill(myP.dpa_lhs) { Vec.fill(myP.dpa_rhs) {
//    Module(new PipelinedDualPortBRAM(
//      addrBits = 1, dataBits = myP.accWidth, regIn = 0, regOut = 0
//    )).io
//  }}
  val res = Module(new StandAloneP2SAccel(myP, p)).io
  res.start := io.start
  io.done := res.done
/*  res.csr <> io.csr
  for(lhs <- 0 until myP.dpa_lhs) {
    for(rhs <- 0 until myP.dpa_rhs) {
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
  io.memPort(0).memWrRsp <> res.dram.wr_rsp*/

  // the signature can be e.g. used for checking that the accelerator has the
  // correct version. here the signature is regenerated from the current date.
  io.signature := makeDefaultSignature()
}
