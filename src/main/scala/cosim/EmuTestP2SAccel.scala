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
    m: Int, n: Int, o: Int, r: Int, unroll: Boolean, p: PlatformWrapperParams
  ) extends GenericAccelerator(p) {
  val numMemPorts = 1
  // parameters for accelerator instance
  val myP = new StandAloneP2SParams(maxInBw = m, nInElemPerWord = n, outStreamSize = o,
    staticSUUnroll = unroll, unrSU = m / r,
    mrp = PYNQZ1Params.toMemReqParams()
  )

  val io = new GenericAcceleratorIF(numMemPorts, p) {
    // base control signals
    val start = Bool(INPUT) // hold high while running
    val done = Bool(OUTPUT) // high when done until start=0
    val cmdqueue = Decoupled(new P2SCmdIO(myP.p2sparams)).flip
    val ackqueue = Decoupled(Bool())
  }

  val accel = Module(new StandAloneP2SAccel(myP, p)).io

  FPGAQueue(io.cmdqueue, 256) <> accel.p2sCmd
  val start_r = Reg(init = Bool(false), next = io.start)
  accel.p2sCmd.valid := io.start & !start_r
  FPGAQueue(accel.ack, 256) <> io.ackqueue
  io.memPort(0) <> accel.memPort(0)
  /*when(io.memPort(0).memRdReq.valid){
    printf("[CO-HW: P2SAlone Received valid request addr %d \n", io.memPort(0).memRdReq.bits.addr )
  }*/
  //when(io.memPort(0).memRdRsp.valid){
  //  printf("[CO-HW: P2SAlone Read Rsp valid, Data %d\n", io.memPort(0).memRdRsp.bits.readData)
  //}

  io.signature := makeDefaultSignature()
}
