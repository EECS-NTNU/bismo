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
  m: Int, n: Int, o: Int, r: Int, unroll: Boolean, p: PlatformWrapperParams) extends GenericAccelerator(p) {
  val numMemPorts = 1
  // parameters for accelerator instance
  val myP = new StandAloneP2SParams(maxInBw = m, nInElemPerWord = n, outStreamSize = o,
    staticSUUnroll = unroll, unrSU = m / r,
    mrp = PYNQZ1Params.toMemReqParams())

  val io = new GenericAcceleratorIF(numMemPorts, p) {
    val enable = Bool(INPUT)
    val cmdqueue = Decoupled(new P2SCmdIO(myP.p2sparams)).flip
    val ackqueue = Decoupled(Bool())
  }

  val accel = Module(new StandAloneP2SAccel(myP, p)).io
  io.memPort(0) <> accel.memPort(0)

  // instantiate and connect cmd and ack queues
  val cmdQ = Module(new FPGAQueue(io.cmdqueue.bits, 16)).io
  val ackQ = Module(new FPGAQueue(io.ackqueue.bits, 16)).io
  io.cmdqueue <> cmdQ.enq
  cmdQ.deq <> accel.p2sCmd
  accel.ack <> ackQ.enq
  ackQ.deq <> io.ackqueue

  // for the accelerator-facing side of the cmd and ack queues,
  // only enable transactions if io.enable is set
  accel.p2sCmd.valid := cmdQ.deq.valid & io.enable
  cmdQ.deq.ready := accel.p2sCmd.ready & io.enable
  accel.ack.ready := ackQ.enq.ready & io.enable
  ackQ.enq.valid := accel.ack.valid & io.enable

  // for the CPU-facing side of the cmd and ack queues,
  // rewire valid/ready with pulse generators to ensure single
  // enqueue/dequeue from the CPU
  cmdQ.enq.valid := io.cmdqueue.valid & !Reg(next=io.cmdqueue.valid)
  ackQ.deq.ready := io.ackqueue.ready & !Reg(next=io.ackqueue.ready)

  io.signature := makeDefaultSignature()
}
