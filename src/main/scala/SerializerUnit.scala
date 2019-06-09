// Copyright (c) 2019 Xilinx
//
// BSD v3 License
//
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// * Redistributions of source code must retain the above copyright notice, this
//   list of conditions and the following disclaimer.
//
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
//
// * Neither the name of BISMO nor the names of its
//   contributors may be used to endorse or promote products derived from
//   this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

package bismo
import Chisel._
import fpgatidbits.synthutils.PrintableParam

class SerializerUnitParams(
  //input precision, how many bits
  //ASSUMING always power of two
  val inPrecision: Int = 32,
  val matrixRows: Int = 8,
  val matrixCols: Int = 8,
  val staticCounter: Boolean = false,
  val staticUnrolling: Boolean = false,
  val unrollingFactor: Int = 1, // unrolling in the input precision dimension
  val maxCounterPrec: Int = 32 // how many pair of bits concatenate
//val clusterFactor : Int = 0
) extends PrintableParam {

  //val outBitWidth : Int = scala.math.pow(2,clusterFactor).toInt
  //val outVectorSize : Int = if(clusterFactor == 0) inPrecision else inPrecision / outBitWidth
  //Impossible to have a cluster factor that try to concatenate more bits than the input bit-width
  // TODO useful to concatenate signals? The != from 1 is to prevent that
  //Predef.assert( (outVectorSize != 0 && outVectorSize != 1) )
  //If not static unroll then single bit serializing per clock cyle otherwise multi-bit serialization of multiple of input precision
  Predef.assert(if (staticCounter && !staticUnrolling) unrollingFactor == 1 else true)
  Predef.assert(if (staticUnrolling) unrollingFactor <= inPrecision else true)
  Predef.assert(if (staticUnrolling) inPrecision % unrollingFactor == 0 else true)

  val actualBitwidth: Int = if (staticCounter && !staticUnrolling) 1 else unrollingFactor
  def headersAsList(): List[String] = {
    return List("InputBitPrecision", "MatrixRows", "MatrixColumns", "staticCounter", "maxCounterPrec")
  }

  def contentAsList(): List[String] = {
    return List(inPrecision, matrixRows, matrixCols, staticCounter, maxCounterPrec).map(_.toString)
  }
}
//TODO update this docs for multi-bit serialization
/**
 * Unit that outputs each clock cycle a bit of each elem of the input matrix
 * @param input a vector of m x n representing the input matrix
 * @param counterValue how much bit of the input should serialize, its value determine the bit-index of the single elem
 * @param start  to increment the value of the counter
 * @param out Decoupled matrix vector ASSUMPTION: ready high means restart to serialize the input, valid high means end of the serialization=
 */
class SerializerUnit(val p: SerializerUnitParams) extends Module {
  val io = new Bundle {
    val input = Vec.fill(p.matrixRows) { Vec.fill(p.matrixCols) { UInt(INPUT, width = p.inPrecision) } }
    //used only for dynamic counter configuration to serialize in a dynamic config
    val counterValue = UInt(INPUT, width = p.maxCounterPrec)
    val start = Bool(INPUT)
    //TODO Digit serializer i.e.: width != 1
    //ASSUMPTION: this decoupled use a different convention: valid end the serialization of the bit-width
    //          whenever the ready signal is high the SU will restart the serialization of the input
    val out = Decoupled(Vec.fill(p.matrixRows) { Vec.fill(p.matrixCols) { Vec.fill(p.actualBitwidth) { UInt(OUTPUT, width = 1) } } })

  }
  val end = Bool()

  //STATIC UNROLLING FOR MULTI-BIT SERIALIZATION  with dynamic maximum count but with static increment
  if (p.staticUnrolling && !p.staticCounter) {
    val myCounterReg = Reg(init = UInt(0x0, width = p.maxCounterPrec))
    val currCount = Reg(init = UInt(0, width = p.maxCounterPrec))

    //not sure about this
    myCounterReg := io.counterValue

    //TODO check if better incrementing this way or multiplying in the index
    when(io.start && currCount < (myCounterReg - UInt(p.unrollingFactor))) {
      currCount := currCount + UInt(p.unrollingFactor)
    }.elsewhen(io.out.ready && (currCount === (myCounterReg - UInt(p.unrollingFactor)) || currCount === myCounterReg)) {
      currCount := UInt(0)
    }.otherwise {
      currCount := currCount
    }

    when(currCount === (myCounterReg - UInt(p.unrollingFactor)) || currCount === myCounterReg) {
      end := Bool(true)
    }.otherwise {
      end := Bool(false)
    }

    for (i ← 0 until p.matrixRows)
      for (j ← 0 until p.matrixCols)
        for (k ← 0 until p.unrollingFactor)
          io.out.bits(i)(j)(k) := io.input(i)(j)(UInt(k) + currCount)

  } //STATIC COUNTER, NOT RECONFIGURABLE AT RUN-TIME
  else if (p.staticCounter) {
    val z = Counter(p.inPrecision)
    when(io.start && z.value < UInt(p.inPrecision - 1) || (io.out.ready && z.value === UInt(p.inPrecision - 1))) {
      z.inc()
    }

    when(z.value === UInt(p.inPrecision - 1)) {
      end := Bool(true)
    }.otherwise {
      end := Bool(false)
    }

    //TODO Right now is only for end of the computation
    io.out.valid := end
    for (i ← 0 until p.matrixRows)
      for (j ← 0 until p.matrixCols)
        io.out.bits(i)(j) := io.input(i)(j)(z.value)

  } // DYNAMIC SOFTWARE CONFIGURABLE COUNTER
  // Same behavior but with a run-time configurable count
  else {
    val z = Module(new DynamicCounter(new DynamicCounterParams(maxPrecision = p.maxCounterPrec))).io
    val myCounterReg = Reg(init = UInt(0xF, width = p.maxCounterPrec))

    when(io.start) {
      myCounterReg := io.counterValue
    }
    z.countMax := myCounterReg
    when(io.start && z.currentValue < (myCounterReg - UInt(1)) || (io.out.ready && z.currentValue === (myCounterReg - UInt(1)))) {
      z.inc := Bool(true)
    }.otherwise {
      z.inc := Bool(false)
    }

    when(z.currentValue === (myCounterReg - UInt(1)) || z.currentValue === myCounterReg) {
      end := Bool(true)
    }.otherwise {
      end := Bool(false)
    }

    for (i ← 0 until p.matrixRows)
      for (j ← 0 until p.matrixCols)
        io.out.bits(i)(j) := io.input(i)(j)(z.currentValue)

  }

  io.out.valid := end

}
