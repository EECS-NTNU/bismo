// Author:  Davide Conficconi
// Date: 17/10/2018
// Revision: 0



package bismo
import fpgatidbits.PlatformWrapper._
import Chisel._
import fpgatidbits.dma._
import fpgatidbits.ocm._

// make the instantiated config options available to softare at runtime
class StandAloneP2SHWCfg(bitsPerField: Int) extends Bundle {
  val readChanWidth = UInt(width = bitsPerField)

  override def cloneType: this.type =
    new StandAloneP2SHWCfg(bitsPerField).asInstanceOf[this.type]
}

// parameters that control the accelerator instantiation
class StandAloneP2SParams(
                             val dpaDimLHS: Int,
                             val mrp: MemReqParams
                           ) extends PrintableParam {
  def estimateResources() = {
    import Math.ceil
    val a_dpu = 2.04
    val b_dpu = 109.41
    val lut_per_res = 120.1
    //val lut_per_dpu = a_dpu * dpaDimCommon + b_dpu
    //val lut_array = dpaDimLHS * dpaDimRHS * (lut_per_dpu + lut_per_res)
    //val bram_array = ceil(dpaDimCommon / 32) * (dpaDimLHS * ceil(lhsEntriesPerMem / 1024) + dpaDimRHS * ceil(rhsEntriesPerMem / 1024))
    //val binops_per_cycle = 2 * dpaDimLHS * dpaDimRHS * dpaDimCommon
    //val tops_per_sec_200MHz = (binops_per_cycle * 200) / 1000000.0
    //println("Resource predictions from cost model")
    //println("=====================================")
    //println(s"DPA LUT: $lut_array")
    //println(s"DPA BRAM: $bram_array")
    //println(s"TOPS at 200 MHz: $tops_per_sec_200MHz")
  }

  def headersAsList(): List[String] = {
    return List(
      "dpaLHS"
    )
  }

  def contentAsList(): List[String] = {
    return List(
      dpaDimLHS
    ).map(_.toString)
  }

  def asHWCfgBundle(bitsPerField: Int): StandAloneP2SHWCfg = {
    val ret = new StandAloneP2SHWCfg(bitsPerField).asDirectionless
    ret.readChanWidth := UInt(mrp.dataWidth)
    //ret.writeChanWidth := UInt(mrp.dataWidth)
    //ret.dpaDimLHS := UInt(dpaDimLHS)

    return ret
  }

}

class StandAloneP2SAccel(
     val myP: StandAloneP2SParams, p: PlatformWrapperParams
     ) extends GenericAccelerator(p) {
  val numMemPorts = 1
  val io = new GenericAcceleratorIF(numMemPorts, p) {
      val hw = new StandAloneP2SHWCfg(32).asOutput()
  }



}
