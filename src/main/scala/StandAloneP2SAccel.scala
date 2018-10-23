// Author:  Davide Conficconi
// Date: 17/10/2018
// Revision: 0



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


  val suparams = new SerializerUnitParams(
    inPrecision = maxInBw, matrixRows = 1,
    matrixCols  = 8, staticCounter = false, maxCounterPrec = maxInBw
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
class dmaIF extends Bundle {
  //val outer_base_addr = UInt(width=32)
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

  val p2skrnl = Module( new P2SKernel(myP.p2sparams)).io
  p2skrnl.ctrl <> io.p2sCtrl
  p2skrnl.start := io.start


  // instantiate request generator
  val inRg = Module(new BlockStridedRqGen( mrp = myP.mrp, writeEn = false )).io

  val start_r = Reg(init = false.B, next = io.start )
  //TODO this is for a single run per start
  val start_pulse = io.start & !start_r
  inRg.in.valid := start_pulse
  /*
  when(inRg.out.valid){
    printf("[HW: SAccel] Valid Read request addr %d\n", inRg.out.bits.addr)
  }
  */
  inRg.in.bits.base :=  p2skrnl.dramBaseSrc
  inRg.in.bits.block_step := io.inDma.outer_step
  inRg.in.bits.block_count := io.inDma.outer_count

  inRg.block_intra_step := io.inDma.inner_step
  inRg.block_intra_count := io.inDma.inner_count


  io.memPort(0).memRdReq <> inRg.out

  //io.memPort(0).memRdReq.bits.numBytes := UInt(myP.maxInBw * myP.nInElemPerWord / 8)

  // add PrintableBundleStreamMonitor to print all mem rd req/rsp transactions
  PrintableBundleStreamMonitor(io.memPort(0).memRdReq, Bool(true), "memRdReq", true)
  //PrintableBundleStreamMonitor(io.memPort(0).memRdRsp, Bool(true), "memRdRsp", true)
  PrintableBundleStreamMonitor(io.memPort(0).memWrReq , Bool(true), "memWrReq", true)
  //PrintableBundleStreamMonitor(io.memPort(0).memWrDat , Bool(true), "memWrDat", true)
  PrintableBundleStreamMonitor(io.memPort(0).memWrRsp , Bool(true), "memWrRsp", true)


/*
  when(inRg.out.valid){
    printf("[HW: SAccel] Addr %d with base addr %d \n", inRg.out.bits.addr, p2skrnl.dramBaseSrc)
  }
  */
  val inQueue = FPGAQueue(ReadRespFilter(io.memPort(0).memRdRsp),128)
  inQueue <> p2skrnl.inputStream
  val regCompletedRdBytes = Reg(init = UInt(0, 32))
  val allRead = regCompletedRdBytes === ( io.inDma.inner_count * UInt(myP.maxInBw * myP.nInElemPerWord) )

//  val rdhandshake = (io.memPort(0).memRdRsp.valid & io.memPort(0).memRdRsp.ready)
  val rdhandshake = (inQueue.valid & p2skrnl.inputStream.ready)


  when(rdhandshake & !allRead){
    regCompletedRdBytes := regCompletedRdBytes + UInt(myP.maxInBw * myP.nInElemPerWord)
  }

/*
  when(io.memPort(0).memRdRsp.valid){
    printf("[HW: SAccel] Valid Read Response: %d\n", io.memPort(0).memRdRsp.bits.readData)

  }
  when(io.memPort(0).memRdRsp.ready && io.memPort(0).memRdRsp.valid ){
    printf("[HW: SAccel] Read Response Handshake!\n")

  }
*/


  val outRg = Module(new BlockStridedRqGen( mrp = myP.mrp, writeEn = true )).io

  outRg.in.valid := allRead

  outRg.in.bits.base := p2skrnl.dramBaseDst
  outRg.in.bits.block_step := io.outDma.outer_step
  outRg.in.bits.block_count := io.outDma.outer_count

  outRg.block_intra_step := io.outDma.inner_step
  outRg.block_intra_count := io.outDma.inner_count

  outRg.out <> io.memPort(0).memWrReq
  //io.memPort(0).memWrReq.bits.numBytes := UInt(myP.p2sparams.outStreamSize / 8)

  FPGAQueue(p2skrnl.outStream, 256) <>  io.memPort(0).memWrDat


  // completion detection logic
  val regCompletedWrBytes = Reg(init = UInt(0, 32))

  val allComplete = (regCompletedWrBytes === io.p2sCtrl.waitCompleteBytes)


  io.memPort(0).memWrRsp.ready := Bool(true)
  when(io.memPort(0).memWrRsp.valid && !allComplete) {
    regCompletedWrBytes := regCompletedWrBytes + UInt(myP.p2sparams.outStreamSize / 8)

  }

  io.done := allComplete




}
