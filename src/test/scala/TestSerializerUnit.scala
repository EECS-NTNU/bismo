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

import Chisel._
import bismo._

import org.scalatest.junit.JUnitSuite
import org.junit.Test
import BISMOTestHelpers._
class TestSerializerUnit extends JUnitSuite {
  //TODO: Update this test for the unrolled version on the output check
  @Test def SerializerUnitTest {
    // Tester-derived class to give stimulus and observe the outputs for the
    // Module to be tested
    class SerializerUnitTester(dut: SerializerUnit) extends Tester(dut) {
      val r = scala.util.Random
      // number of re-runs for each test
      val num_seqs = 5
      val rows = dut.p.matrixRows
      val cols = dut.p.matrixCols

      // number of bits for the input data
      val in_len = dut.p.inPrecision
      val rolling = dut.p.unrollingFactor
      val cycles2Complete = in_len / rolling

      for (i ← 1 to num_seqs) {

        if (!dut.p.staticCounter) {
          poke(dut.io.counterValue, scala.math.BigInt.apply(in_len))
        } else {
          poke(dut.io.counterValue, scala.math.BigInt.apply(0))
        }

        val neg = r.nextBoolean()
        val a = BISMOTestHelpers.randomIntMatrix(rows, cols, in_len, neg)
        println("Matrix a")
        printMatrix(a)
        poke(dut.io.out.ready, false)
        for (i ← 0 until rows)
          for (j ← 0 until cols)
            poke(dut.io.input(i)(j), scala.math.BigInt.apply(a(i)(j)))
        step(1)
        poke(dut.io.start, true)
        for (k ← 0 until in_len /*/cycles2Complete*/ ) {
          for (i ← 0 until rows)
            for (j ← 0 until cols)
              expect(dut.io.out.bits(i)(j)(0), BISMOTestHelpers.extractBitPos(a(i)(j), k, in_len))
          step(1)
        }
        poke(dut.io.out.ready, true)
        poke(dut.io.start, false)
        step(1)
      }
    }

    // Chisel arguments to pass to chiselMainTest
    def testArgs = BISMOTestHelpers.stdArgs

    for {
      inPrecision ← 16 to 16
      rows ← 1 to 1
      cols ← 8 to 8
      static ← List(false)
      countWidth ← 16 to 16

    } {
      // function that instantiates the Module to be tested
      val p = new SerializerUnitParams(
        inPrecision = inPrecision, matrixRows = rows, matrixCols = cols, staticCounter = static, maxCounterPrec = countWidth,
        staticUnrolling = false, unrollingFactor = 2 //inPrecision/2
      )
      def testModuleInstFxn = () ⇒ { Module(new SerializerUnit(p)) }
      // function that instantiates the Tester to test the Module
      def testTesterInstFxn = (dut: SerializerUnit) ⇒ new SerializerUnitTester(dut)
      // actually run the test
      chiselMainTest(
        testArgs,
        testModuleInstFxn
      ) {
          testTesterInstFxn
        }
    }
  }
}
