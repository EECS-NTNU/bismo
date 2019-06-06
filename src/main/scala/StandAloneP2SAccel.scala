/**
 * Stand Alone Accelerator of P2S component, a single run is assumed to completely serialize matrix of consecutives bytes
 */

package bismo
import fpgatidbits.PlatformWrapper._
import Chisel._
import fpgatidbits.dma._
import fpgatidbits.ocm._
import fpgatidbits.streams.{ PrintableBundle, PrintableBundleStreamMonitor, ReadRespFilter }
import fpgatidbits.synthutils.PrintableParam

// parameters that control the accelerator instantiation
class StandAloneP2SParams(
  val maxInBw: Int,
  val nInElemPerWord: Int,
  val outStreamSize: Int,
  val fastMode: Boolean = true,
  val mrp: MemReqParams) extends PrintableParam {

  // Input bandwidth equal to output bandwidth
  Predef.assert(maxInBw * nInElemPerWord == outStreamSize)
  Predef.assert(maxInBw % 8 == 0 || nInElemPerWord % 8 == 0)

  val dramWordBytes = maxInBw * nInElemPerWord / 8

  // number of columns given to the StandAloneP2SAccel must be an integer multiple
  // of p2sAlign. in other words, this is the "number of columns in a group".
  val p2sAlign = maxInBw * nInElemPerWord

  val p2sparams = new P2SKernelParams(
    maxInBw = maxInBw, nInElemPerWord = nInElemPerWord,
    outStreamSize = outStreamSize, mrp = mrp)
  def headersAsList(): List[String] = {
    return List(
      "M-axInBw", "N-InElemPerWord", "O-utStreamSize", "F-astMode")
  }

  def contentAsList(): List[String] = {
    return List(
      maxInBw, nInElemPerWord, outStreamSize, fastMode).map(_.toString)
  }
}

class StreamingSignCorrection(myP: StandAloneP2SParams) extends Module {
  val io = new Bundle {
    val actualPrecision = UInt(INPUT, width = log2Up(myP.maxInBw) + 1)
    val signed = Bool(INPUT)
    val in = Decoupled(UInt(width = myP.outStreamSize)).flip()
    val out = Decoupled(UInt(width = myP.outStreamSize))
  }

  // TODO compute the value of the mask for other cases
  Predef.assert(myP.maxInBw == 8)
  Predef.assert(myP.nInElemPerWord == 8)
  val sign_bitmask = UInt("h8080808080808080", width = myP.outStreamSize)
  val sign_actual_prec = (io.in.bits & sign_bitmask) >> (UInt(myP.maxInBw) - io.actualPrecision)
  io.out <> io.in
  // override output bits depending on signedness
  io.out.bits := Mux(io.signed, io.in.bits | sign_actual_prec, io.in.bits)
  /*when(io.out.fire()) {
    printf("[SignCorrection] signed? %d prec %x in %x out %x\n", io.signed, io.actualPrecision, io.in.bits, io.out.bits)
  }*/
}

class P2SCmdIO(myP: P2SKernelParams) extends PrintableBundle {
  // DRAM base address for source (bit parallel) matrix
  val dramBaseAddrSrc = UInt(width = 32)
  // DRAM base address for destination (bit serial) matrix
  val dramBaseAddrDst = UInt(width = 32)
  // number of matrix rows
  val matrixRows = UInt(width = 32)
  // number of matrix column groups
  // one column group = maxInBw * nInElemPerWord elements
  val matrixColsGroup = UInt(width = 32)
  // actual precision of the input bit-parallel matrix, <= maxInBw
  // this field must be able to represent maxInBw, hence the +1
  val actualPrecision = UInt(width = log2Up(myP.maxInBw) + 1)
  // total size of destination (bit serial) matrix in bytes
  val waitCompleteBytes = UInt(width = 32)
  // signedness (moves sign bit from maxInBw to actualPrecision)
  val signed = Bool()

  override def cloneType(): this.type =
    new P2SCmdIO(myP).asInstanceOf[this.type]
  val printfStr = "DRAM {src: %x, dst: %x} matrix {rows %d, colgroups %d, bits %d} waitCompleteBytes %d signed %d\n"
  val printfElems = { () ⇒
    Seq(
      dramBaseAddrSrc, dramBaseAddrDst, matrixRows, matrixColsGroup,
      actualPrecision, waitCompleteBytes, signed)
  }
}

class StandAloneP2SAccel(
  val myP: StandAloneP2SParams, p: PlatformWrapperParams) extends Module() { //GenericAccelerator(p) {
  val numMemPorts = 1
  val io = new GenericAcceleratorIF(numMemPorts, p) {
    val p2sCmd = Decoupled(new P2SCmdIO(myP.p2sparams)).flip()
    // clock cycle count for each command is returned on completion
    val ack = Decoupled(UInt(width = 32))
  }
  // register the p2sCmd currently being processed
  val regCmd = Reg(outType = io.p2sCmd.bits)

  // instantiate the actual P2SKernel and request generators
  val readRg = Module(new BlockStridedRqGen(
    mrp = myP.mrp, writeEn = false, chanID = BISMOLimits.p2sDRAMChanID
  )).io
  val writeRg = Module(new BlockStridedRqGen(
    mrp = myP.mrp, writeEn = true, chanID = BISMOLimits.p2sDRAMChanID
  )).io
  val p2skrnl = Module(
    if (myP.fastMode) { new P2SKernel_Fast(myP.p2sparams) }
    else { new P2SKernel_Slow(myP.p2sparams) }).io
  p2skrnl.actualPrecision := regCmd.actualPrecision

  // write completion detection logic
  val regCompletedWrBytes = Reg(init = UInt(0, 32))
  // note: waitCompleteBytes must be a multiple of mrp.dataWidth in bytes
  val writeComplete = (regCompletedWrBytes === regCmd.waitCompleteBytes)
  io.memPort(0).memWrRsp.ready := Bool(true)
  when(io.memPort(0).memWrRsp.fire()) {
    // TODO: this needs to take burst width into account, if any
    regCompletedWrBytes := regCompletedWrBytes + UInt(myP.mrp.dataWidth / 8)
  }

  // FSM logic for control for starting the P2S processing
  val sIdle :: sGenReads :: sGenWrites :: sWaitComplete :: sGenAck :: Nil = Enum(UInt(), 5)
  val regState = Reg(init = sIdle)

  //Default values
  io.p2sCmd.ready := Bool(false)
  readRg.in.valid := Bool(false)
  writeRg.in.valid := Bool(false)
  io.ack.valid := Bool(false)

  // cycle counter for performance measurements
  val regCycleCount = Reg(init = UInt(0, width = 32))
  when(regState != sIdle) {
    regCycleCount := regCycleCount + UInt(1)
  }
  io.ack.bits := regCycleCount

  switch(regState) {
    is(sIdle) {
      regCompletedWrBytes := UInt(0)
      regCycleCount := UInt(0)
      io.p2sCmd.ready := Bool(true)
      when(io.p2sCmd.valid) {
        regCmd := io.p2sCmd.bits
        regState := sGenReads
      }
    }
    is(sGenReads) {
      readRg.in.valid := Bool(true)
      when(readRg.in.ready) {
        regState := sGenWrites
      }
    }
    is(sGenWrites) {
      writeRg.in.valid := Bool(true)
      when(writeRg.in.ready) {
        regState := sWaitComplete
      }
    }
    is(sWaitComplete) {
      when(writeComplete) {
        regState := sGenAck
      }
    }
    is(sGenAck) {
      io.ack.valid := Bool(true)
      when(io.ack.ready) {
        regState := sIdle
      }
    }
  }

  readRg.block_intra_step := UInt(myP.dramWordBytes)
  // number of steps within block = number of DRAM words per row
  // = (number of column groups) * (elements per group) / (elements per DRAM word)
  // since elements per group = maxInBw * elements per word:
  // = number of column groups * maxInBw
  readRg.block_intra_count := UInt(myP.maxInBw) * regCmd.matrixColsGroup //* UInt(myP.maxInBw)

  // at the moment reads are actually on a single continuous block, so the
  // block-inter step here is simply the product of block-intra step and count
  readRg.in.bits.base := regCmd.dramBaseAddrSrc
  readRg.in.bits.block_step := regCmd.matrixColsGroup * UInt(myP.maxInBw) * UInt(myP.dramWordBytes)
  //continuous run? or just single row?
  readRg.in.bits.block_count := regCmd.matrixRows
  //  inRg.in.bits.block_count := UInt(1)

  io.memPort(0).memRdReq <> readRg.out
  val ssc = Module(new StreamingSignCorrection(myP)).io
  ssc.actualPrecision := regCmd.actualPrecision
  ssc.signed := regCmd.signed

  FPGAQueue(ReadRespFilter(io.memPort(0).memRdRsp), 4) <> ssc.in
  ssc.out <> p2skrnl.inputStream

/*****************************DEBUG PRINT********************************************/
  // add PrintableBundleStreamMonitor to print all mem rd req/rsp transactions
  //  PrintableBundleStreamMonitor(io.memPort(0).memRdReq, Bool(true), "memRdReq", true)
  //  PrintableBundleStreamMonitor(io.memPort(0).memRdRsp, Bool(true), "memRdRsp", true)
  //  PrintableBundleStreamMonitor(io.memPort(0).memWrReq , Bool(true), "memWrReq", true)
  //  when(io.memPort(0).memWrDat.ready & io.memPort(0).memWrDat.valid ){
  //    printf("[HW: P2S Accel] Sending this data %x\n", io.memPort(0).memWrDat.bits)
  //  }
  //PrintableBundleStreamMonitor(io.memPort(0).memWrDat , Bool(true), "memWrDat", true)
  //PrintableBundleStreamMonitor(io.memPort(0).memWrRsp , Bool(true), "memWrRsp", true)

  writeRg.in.bits.base := regCmd.dramBaseAddrDst
  writeRg.in.bits.block_step := UInt(myP.dramWordBytes)
  writeRg.in.bits.block_count := regCmd.matrixColsGroup * regCmd.matrixRows

  writeRg.block_intra_step := regCmd.matrixColsGroup * regCmd.matrixRows * UInt(myP.dramWordBytes)
  writeRg.block_intra_count := regCmd.actualPrecision

  val outAddrQueue = FPGAQueue(writeRg.out, 4)
  outAddrQueue <> io.memPort(0).memWrReq
  io.memPort(0).memWrReq.bits.numBytes := UInt(myP.dramWordBytes)

  val dataQueue = FPGAQueue(p2skrnl.outStream, 4)

  dataQueue <> io.memPort(0).memWrDat
}
