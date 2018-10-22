// Author:  Davide Conficconi
// Date: 18/10/2018
// Revision: 0

package bismo

import Chisel._
import fpgatidbits.ocm._
import fpgatidbits.dma._
import fpgatidbits.streams._
import fpgatidbits.PlatformWrapper._



class EmuTestP2SAccel(
                       m: Int, n: Int,  o: Int, p: PlatformWrapperParams
    ) extends GenericAccelerator(p) {
  val numMemPorts = 1
  // parameters for accelerator instance
  val myP = new StandAloneP2SParams( maxInBw = m, nInElemPerWord = n, outStreamSize = o,
   mrp = PYNQZ1Params.toMemReqParams()
  )

  val io = new GenericAcceleratorIF(numMemPorts, p) {
    // base control signals
    val start = Bool(INPUT)                   // hold high while running
    val done = Bool(OUTPUT)                   // high when done until start=0
    val csr = new P2SKernelCtrlIO(myP.p2sparams).asInput()
    val inDma = new dmaIF().asInput()
    val outDma = new dmaIF().asInput()
  }

  val accel = Module(new StandAloneP2SAccel(myP, p)).io
  accel.start := io.start
  io.done := accel.done
  accel.p2sCtrl <> io.csr
  io.memPort(0) <> accel.memPort(0)
  /*when(io.memPort(0).memRdReq.valid){
    printf("[CO-HW: P2SAlone Received valid request addr %d \n", io.memPort(0).memRdReq.bits.addr )
  }*/
  //when(io.memPort(0).memRdRsp.valid){
  //  printf("[CO-HW: P2SAlone Read Rsp valid, Data %d\n", io.memPort(0).memRdRsp.bits.readData)
  //}
  io.inDma <> accel.inDma
  io.outDma <> accel.outDma


  io.signature := makeDefaultSignature()
}
