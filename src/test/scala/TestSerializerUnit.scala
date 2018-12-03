// Author:  Davide Conficconi
// Date: 23/08/2018
// Revision: 0
import Chisel._
import bismo._

import org.scalatest.junit.JUnitSuite
import org.junit.Test
import RosettaTestHelpers._
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
        val a = RosettaTestHelpers.randomIntMatrix(rows, cols, in_len, neg)
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
              expect(dut.io.out.bits(i)(j)(0), RosettaTestHelpers.extractBitPos(a(i)(j), k, in_len))
          step(1)
        }
        poke(dut.io.out.ready, true)
        poke(dut.io.start, false)
        step(1)
      }
    }

    // Chisel arguments to pass to chiselMainTest
    def testArgs = RosettaTestHelpers.stdArgs

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
