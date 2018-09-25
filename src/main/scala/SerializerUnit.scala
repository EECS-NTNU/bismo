// Author:  Davide Conficconi
// Date: 21/09/2018
// Revision: 0

package bismo
import Chisel._


class SerializerUnitParams(
      //input precision, how many bits
      //ASSUMING always power of two
      val inPrecision : Int = 32,
      val matrixRows : Int = 8,
      val matrixCols : Int = 8,
      val staticCounter : Boolean = false,
      val maxCounterPrec : Int = 32
      // how many pair of bits concatenate
      //val clusterFactor : Int = 0
      ) extends PrintableParam {

  //val outBitWidth : Int = scala.math.pow(2,clusterFactor).toInt
  //val outVectorSize : Int = if(clusterFactor == 0) inPrecision else inPrecision / outBitWidth
  //Impossible to have a cluster factor that try to concatenate more bits than the input bit-width
  // TODO useful to concatenate signals? The != from 1 is to prevent that
  //Predef.assert( (outVectorSize != 0 && outVectorSize != 1) )

  def headersAsList(): List[String] = {
    return List("InputBitPrecision", "MatrixRows", "MatrixColumns")
  }

  def contentAsList(): List[String] = {
    return List(inPrecision, matrixRows, matrixCols).map(_.toString)
  }
}

class SerializerUnit(val p : SerializerUnitParams) extends Module{
  val io = new Bundle {
    val input = Vec.fill(p.matrixRows){Vec.fill(p.matrixCols){UInt(INPUT, width = p.inPrecision)}}
    //used only for dynamic counter configuration to serialize in a dynamic config
    val counterValue = UInt(INPUT, width = p.maxCounterPrec)
    val start = Bool(INPUT)
    //TODO Digit serializer i.e.: width != 1
    val out = Decoupled(Vec.fill(p.matrixRows){Vec.fill(p.matrixCols){UInt(OUTPUT, width = 1)}})

  }
  val end = Bool()
  //STATIC COUNTER, NOT RECONFIGURABLE AT RUN-TIME
  if (p.staticCounter){
    val z = Counter(p.inPrecision)
    when(io.start && z.value < UInt(p.inPrecision-1) ||( io.out.ready && z.value === UInt(p.inPrecision - 1)) ){
        z.inc()
    }

    when(z.value === UInt(p.inPrecision - 1)){
      end := Bool(true)
    }.otherwise{
      end := Bool(false)
    }

    //TODO Right now is only for end of the computation
    io.out.valid := end

    for (i <- 0 until p.matrixRows)
      for (j <- 0 until p.matrixCols)
        io.out.bits(i)(j) := io.input(i)(j)(z.value)

  }
  // DYNAMIC SOFTWARE CONFIGURABLE COUNTER
  // Same behavior but with a run-time configurable count
  else {
    val z = Module(new DynamicCounter( new DynamicCounterParams( maxPrecision =  p.maxCounterPrec))).io
    z.countMax := io.counterValue

    when(io.start && z.currentValue < UInt(p.inPrecision-1) ||( io.out.ready && z.currentValue === UInt(p.inPrecision - 1)) ){
      z.inc := Bool(true)
    }.otherwise{
      z.inc := Bool(false)
    }

    when(z.currentValue === UInt(p.inPrecision - 1)){
      end := Bool(true)
    }.otherwise{
      end := Bool(false)
    }

    for (i <- 0 until p.matrixRows)
      for (j <- 0 until p.matrixCols)
        io.out.bits(i)(j) := io.input(i)(j)(z.currentValue)

  }

  io.out.valid := end


}



