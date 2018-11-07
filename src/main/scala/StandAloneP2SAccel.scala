// Author:  Davide Conficconi
// Date: 17/10/2018
// Revision: 0

/**
  * Stand Alone Accelerator of P2S component, a single run is assumed to completely serialize matrix of consecutives bytes
  */

package bismo
import fpgatidbits.PlatformWrapper._
import Chisel._
import fpgatidbits.dma._
import fpgatidbits.ocm._
import fpgatidbits.streams.{PrintableBundle, PrintableBundleStreamMonitor, ReadRespFilter}


// parameters that control the accelerator instantiation
class StandAloneP2SParams(
                             val maxInBw: Int,
                             val nInElemPerWord : Int,
                             val outStreamSize : Int,
                             val staticCntr : Boolean = false,
                             val staticSUUnroll : Boolean = false,
                             val unrSU : Int = 1,
                             val mrp: MemReqParams
                           ) extends PrintableParam {

  // Input bandwidth equal to output bandwidth
  Predef.assert( maxInBw * nInElemPerWord == outStreamSize)
  Predef.assert(maxInBw % 8 == 0 || nInElemPerWord % 8 == 0)

  val dramWordBytes = maxInBw * nInElemPerWord / 8

  val suparams = new SerializerUnitParams(
    inPrecision = maxInBw, matrixRows = 1,
    matrixCols  = nInElemPerWord, staticCounter = staticCntr,
    maxCounterPrec = maxInBw, staticUnrolling = staticSUUnroll,
    unrollingFactor = unrSU
  )

  val p2sparams =   new P2SKernelParams(
    maxInBw = maxInBw, nInElemPerWord = nInElemPerWord,
    outStreamSize  = outStreamSize, mrp = mrp,
    suparams = suparams
  )
  def headersAsList(): List[String] = {
    return List(
      "M-axInBw", "N-InElemPerWord", "O-utStreamSize", "SU-StatiCounter", "SU-Static Unroll", "SU-Unrolling factor"
    )
  }

  def contentAsList(): List[String] = {
    return List(
      maxInBw, nInElemPerWord,outStreamSize,staticCntr,
      staticSUUnroll,unrSU
    ).map(_.toString)
  }
}

// Configuration for a Block strided Request generation, the step is intended for byte-addressable memory (e.g. 8, 16 ...)
class dmaIF extends Bundle {
  val outer_step = UInt(width=32)
  val outer_count = UInt(width =  32)
  val inner_step = UInt(width = 32)
  val inner_count = UInt(width = 32)
}

class P2SCmdIO(myP: P2SKernelParams) extends PrintableBundle{
  val dramBaseAddrSrc = UInt(width = 32)
  val dramBaseAddrDst = UInt(width = 32)
  val matrixRows = UInt(width = 32)
  val matrixColsGroup = UInt(width = 32)//how many columns groups, should always be %64 == 0
  val actualPrecision = UInt(width = myP.maxInBw)
  val waitCompleteBytes = UInt(width = 32)

  override def cloneType(): this.type =
    new P2SCmdIO(myP).asInstanceOf[this.type]
  val printfStr = "DRAM base src addr: %d, dst addr: %d, Matrix size: %d, %d, with current precision of %d\n"
  val printfElems = {()=> Seq(dramBaseAddrSrc, dramBaseAddrDst, matrixRows, matrixColsGroup, actualPrecision)}
}

class StandAloneP2SAccel(
     val myP: StandAloneP2SParams, p: PlatformWrapperParams
     ) extends Module(){//GenericAccelerator(p) {
  val numMemPorts = 1
  val io = new GenericAcceleratorIF(numMemPorts, p) {
      val p2sCmd = Decoupled(new P2SCmdIO(myP.p2sparams)).flip()
      val ack = Decoupled(Bool())
  }


  val start = Bool()
  val done = Bool()




  // instantiate request generator
  val readRg = Module(new BlockStridedRqGen( mrp = myP.mrp, writeEn = false )).io
  val writeRg = Module(new BlockStridedRqGen( mrp = myP.mrp, writeEn = true )).io

  val p2skrnl = Module( new P2SKernel(myP.p2sparams)).io
  p2skrnl.actualPrecision := io.p2sCmd.bits.actualPrecision



  // write completion detection logic
  val regCompletedWrBytes = Reg(init = UInt(0, 32))
  //ATTENTION: this should be a multiple of 8
  val writeComplete = regCompletedWrBytes === io.p2sCmd.bits.waitCompleteBytes
  io.memPort(0).memWrRsp.ready := Bool(true)
  when(io.memPort(0).memWrRsp.valid && !writeComplete) {
    regCompletedWrBytes := regCompletedWrBytes + UInt(myP.p2sparams.outStreamSize / 8)

  }


  // FSM logic for control for starting the P2S processing
  val sIdle :: sGenReads :: sGenWrites :: sWaitComplete :: sGenAck :: Nil = Enum(UInt(), 5)
  val regState = Reg(init = sIdle)

  //Default values
  io.p2sCmd.ready := Bool(false)
  readRg.in.valid := Bool(false)
  writeRg.in.valid := Bool(false)
  io.ack.valid := Bool(false)
  io.ack.bits := Bool(false)

  switch(regState){
    is (sIdle){
      io.p2sCmd.ready := Bool(true)
      when(io.p2sCmd.valid){
        regState := sGenReads
      }
    }
    is (sGenReads){
      readRg.in.valid := Bool(true)
      regState := sGenWrites

    }
    is (sGenWrites){
      writeRg.in.valid := Bool(true)
      regState := sWaitComplete
    }
    is (sWaitComplete){
      when(writeComplete){
        regState := sGenAck
      }
    }
    is (sGenAck){
      io.ack.valid := Bool(true)
      io.ack.bits := Bool(true)
      regState := sIdle
      regCompletedWrBytes := UInt(0)
    }

  }


  readRg.in.bits.base := io.p2sCmd.bits.dramBaseAddrSrc

  readRg.in.bits.block_step := io.p2sCmd.bits.matrixColsGroup * io.p2sCmd.bits.actualPrecision * UInt(myP.dramWordBytes)

  //continuous run? or just single row?
  readRg.in.bits.block_count := io.p2sCmd.bits.matrixRows
//  inRg.in.bits.block_count := UInt(1)

  readRg.block_intra_step := UInt(myP.dramWordBytes)
  readRg.block_intra_count := io.p2sCmd.bits.matrixColsGroup * io.p2sCmd.bits.actualPrecision


  io.memPort(0).memRdReq <> readRg.out

  FPGAQueue(ReadRespFilter(io.memPort(0).memRdRsp),256) <> p2skrnl.inputStream

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


  writeRg.in.bits.base := io.p2sCmd.bits.dramBaseAddrDst
  writeRg.in.bits.block_step := UInt(myP.dramWordBytes)
  writeRg.in.bits.block_count := io.p2sCmd.bits.matrixColsGroup * io.p2sCmd.bits.matrixRows

  writeRg.block_intra_step := io.p2sCmd.bits.matrixColsGroup * io.p2sCmd.bits.matrixRows * UInt(myP.dramWordBytes)
  writeRg.block_intra_count := io.p2sCmd.bits.actualPrecision

  val outAddrQueue = FPGAQueue(writeRg.out, 256)
  outAddrQueue <> io.memPort(0).memWrReq
  io.memPort(0).memWrReq.bits.numBytes := UInt(myP.dramWordBytes)

  val dataQueue = FPGAQueue(p2skrnl.outStream, 256)
  dataQueue <>  io.memPort(0).memWrDat

}
