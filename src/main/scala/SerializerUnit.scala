// Author:  Davide Conficconi
// Date: 21/09/2018
// Revision: 0

package bismo
import Chisel._


class SerializerUnitParams(
      //input precision, how many bits
      //ASSUMING always power of two
      val inPrecision : Int = 32,
      val mRows : Int = 8,
      val mCols : Int = 8
      // how many pair of bits concatenate
      //val clusterFactor : Int = 0
      ) extends PrintableParam {

  //val outBitWidth : Int = scala.math.pow(2,clusterFactor).toInt
  //val outVectorSize : Int = if(clusterFactor == 0) inPrecision else inPrecision / outBitWidth
  //Impossible to have a cluster factor that try to concatenate more bits than the input bit-width
  // TODO useful to concatenate signals? The != from 1 is to prevent that
  //Predef.assert( (outVectorSize != 0 && outVectorSize != 1) )

  def headersAsList(): List[String] = {
    return List("InputBitPrecision")
  }

  def contentAsList(): List[String] = {
    return List(inPrecision).map(_.toString)
  }
}

class SerializerUnit(val p : SerializerUnitParams) extends Module{
  val io = new Bundle {
    val input = Vec.fill(p.mRows){Vec.fill(p.mCols){UInt(INPUT, width = p.inPrecision)}}
    val start = Bool(INPUT)
    //TODO Digit serializer i.e.: width != 1
    val out = Decoupled(Vec.fill(p.mRows){Vec.fill(p.mCols){UInt(OUTPUT, width = 1)}})

  }
  //val inReg =
  val z = Counter(p.inPrecision)


  when(io.start && z.value < UInt(p.inPrecision-1) ||( io.out.ready && z.value === UInt(p.inPrecision - 1)) ){
    z.inc()
  }

  val end = Bool()
  when(z.value === UInt(p.inPrecision - 1)){
    end := Bool(true)
  }.otherwise{
    end := Bool(false)
  }

  //TODO Right now is only for end of the computation
  io.out.valid := end

  for (i <- 0 until p.mRows)
    for (j <- 0 until p.mCols)
      io.out.bits(i)(j) := io.input(i)(j)(z.value)

}



