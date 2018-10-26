// Author:  Davide Conficconi
// Date: 17/10/2018
// Revision: 0

/**
  * Stand Alone Accelerator of P2S component, a single run is assumed to completely serialize consecutives bytes
  */

package bismo
import fpgatidbits.PlatformWrapper._
import Chisel._
import fpgatidbits.dma._
import fpgatidbits.ocm._
import fpgatidbits.streams.ReadRespFilter
import fpgatidbits.streams.PrintableBundleStreamMonitor

// make the instantiated config options available to softare at runtime
class StandAloneP2SHWCfg(bitsPerField: Int) extends Bundle {
  val readChanWidth = UInt(width = bitsPerField)
  val writeChanWidth = UInt(width = bitsPerField)


  override def cloneType: this.type =
    new StandAloneP2SHWCfg(bitsPerField).asInstanceOf[this.type]
}

// parameters that control the accelerator instantiation
class StandAloneP2SParams(
                             val maxInBw: Int,
                             val nInElemPerWord : Int,
                             val outStreamSize : Int,
                             val mrp: MemReqParams
                           ) extends PrintableParam {

  // Input bandwidth equal to output bandwidth
  Predef.assert( maxInBw * nInElemPerWord == outStreamSize)
  Predef.assert(maxInBw %8 == 0 || nInElemPerWord % 8 == 0)

  val dramWordBytes = maxInBw * nInElemPerWord / 8

  val suparams = new SerializerUnitParams(
    inPrecision = maxInBw, matrixRows = 1,
    matrixCols  = nInElemPerWord, staticCounter = false, maxCounterPrec = maxInBw
  )

  val p2sparams =   new P2SKernelParams(
    maxInBw = maxInBw, nInElemPerWord = nInElemPerWord,
    outStreamSize  = outStreamSize, mrp = mrp,
    suparams = suparams
  )

  def headersAsList(): List[String] = {
    return List(
      "maxInBw", "nInElemPerWord", "outStreamSize"
    )
  }

  def contentAsList(): List[String] = {
    return List(
      maxInBw, nInElemPerWord,outStreamSize
    ).map(_.toString)
  }

  def asHWCfgBundle(bitsPerField: Int): StandAloneP2SHWCfg = {
    val ret = new StandAloneP2SHWCfg(bitsPerField).asDirectionless
    ret.readChanWidth := UInt(mrp.dataWidth)
    ret.writeChanWidth := UInt(mrp.dataWidth)
    //ret.dpaDimLHS := UInt(dpaDimLHS)

    return ret
  }

}

// Configuration for a Block strided Request generation, the step is intended for byte-addressable memory (e.g. 8, 16 ...)
class dmaIF extends Bundle {
  val outer_step = UInt(width=32)
  val outer_count = UInt(width =  32)
  val inner_step = UInt(width = 32)
  val inner_count = UInt(width = 32)
}

class StandAloneP2SAccel(
     val myP: StandAloneP2SParams, p: PlatformWrapperParams
     ) extends GenericAccelerator(p) {
  val numMemPorts = 1
  val io = new GenericAcceleratorIF(numMemPorts, p) {
      val hw = new StandAloneP2SHWCfg(32).asOutput()
      val inDma = new dmaIF().asInput()
      val outDma = new dmaIF().asInput()
      val p2sCtrl = new P2SKernelCtrlIO(myP.p2sparams).asInput()
      val start = Bool(INPUT)
      val done = Bool(OUTPUT)
  }



  val colPerWord = io.p2sCtrl.matrixCols * UInt(myP.dramWordBytes)

  // instantiate request generator
  val inRg = Module(new BlockStridedRqGen( mrp = myP.mrp, writeEn = false )).io

  val start_r = Reg(init = false.B, next = io.start )
  //TODO this is for a single run per start
  val start_pulse = io.start & !start_r
  inRg.in.valid := start_pulse


  val p2skrnl = Module( new P2SKernel(myP.p2sparams)).io
  p2skrnl.ctrl <> io.p2sCtrl
  p2skrnl.start := start_pulse


  val bitParallelColPerDramWord = (io.p2sCtrl.matrixCols * io.p2sCtrl.actualInBw ) / UInt(myP.maxInBw * myP.nInElemPerWord)
  val sel_bprl = bitParallelColPerDramWord > UInt(1)
  val bitPrlCount = Mux(sel_bprl,bitParallelColPerDramWord, UInt(1) )

  inRg.in.bits.base :=  p2skrnl.dramBaseSrc
  inRg.in.bits.block_step := UInt(1)//io.p2sCtrl.matrixRows * io.p2sCtrl.matrixCols *  UInt(myP.dramWordBytes) // ext param? for multiple rows

  //TODO check this and upper
  inRg.in.bits.block_count := UInt(1)

  inRg.block_intra_step := UInt(myP.dramWordBytes)
  inRg.block_intra_count := bitPrlCount // max(col * bw / dram word, 1), should it be an instruction param?


  p2skrnl.multipleExec := sel_bprl
  io.memPort(0).memRdReq <> inRg.out


  // add PrintableBundleStreamMonitor to print all mem rd req/rsp transactions
//  PrintableBundleStreamMonitor(io.memPort(0).memRdReq, Bool(true), "memRdReq", true)
//  PrintableBundleStreamMonitor(io.memPort(0).memRdRsp, Bool(true), "memRdRsp", true)
  //PrintableBundleStreamMonitor(io.memPort(0).memWrReq , Bool(true), "memWrReq", true)
  //PrintableBundleStreamMonitor(io.memPort(0).memWrDat , Bool(true), "memWrDat", true)
  //PrintableBundleStreamMonitor(io.memPort(0).memWrRsp , Bool(true), "memWrRsp", true)



  val inQueue = FPGAQueue(ReadRespFilter(io.memPort(0).memRdRsp),128)
  inQueue <> p2skrnl.inputStream

  val regCompletedRdBytes = Reg(init = UInt(0, 32))

  val allRead = regCompletedRdBytes === ( bitPrlCount * UInt(myP.maxInBw * myP.nInElemPerWord) )

//  val rdhandshake = (io.memPort(0).memRdRsp.valid & io.memPort(0).memRdRsp.ready)
  val rdhandshake = inQueue.valid & p2skrnl.inputStream.ready


  when(rdhandshake & !allRead){
    regCompletedRdBytes := regCompletedRdBytes + UInt(myP.maxInBw * myP.nInElemPerWord)
  }


  val outRg = Module(new BlockStridedRqGen( mrp = myP.mrp, writeEn = true )).io

  outRg.in.valid := p2skrnl.done


  val bsColPerDramWord = io.p2sCtrl.matrixCols / UInt(myP.dramWordBytes)
  val sel_bs = bsColPerDramWord > UInt(myP.dramWordBytes)

  val stepFactor =  Mux(sel_bs, bsColPerDramWord, UInt(myP.dramWordBytes) )
  val intStep = io.p2sCtrl.matrixRows * stepFactor


  val sel_count = bsColPerDramWord > UInt(1)

  val ext_count = Mux(sel_count, bsColPerDramWord, UInt(1) )

  outRg.in.bits.base := p2skrnl.dramBaseDst // should always been the address of b0 current processed row
  outRg.in.bits.block_step := UInt(myP.dramWordBytes)
  outRg.in.bits.block_count := ext_count

  outRg.block_intra_step := intStep
  outRg.block_intra_count := io.p2sCtrl.actualInBw

  outRg.out <> io.memPort(0).memWrReq


  FPGAQueue(p2skrnl.outStream, 64) <>  io.memPort(0).memWrDat


  // completion detection logic
  val regCompletedWrBytes = Reg(init = UInt(0, 32))

  val allComplete = regCompletedWrBytes === io.p2sCtrl.waitCompleteBytes

  io.memPort(0).memWrRsp.ready := Bool(true)
  when(io.memPort(0).memWrRsp.valid && !allComplete) {
    regCompletedWrBytes := regCompletedWrBytes + UInt(myP.p2sparams.outStreamSize / 8)

  }

  io.done := allComplete




}
