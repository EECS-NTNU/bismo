// Author:  Davide Conficconi
// Date: 23/08/2018
// Revision: 0
import Chisel._
import bismo._

import org.scalatest.junit.JUnitSuite
import org.junit.Test
import RosettaTestHelpers._
class TestSerializerUnit extends JUnitSuite {
  @Test def SerializerUnitTest {
    // Tester-derived class to give stimulus and observe the outputs for the
    // Module to be tested
    class SerializerUnitTester(dut: SerializerUnit) extends Tester(dut) {
      val r = scala.util.Random
      // number of re-runs for each test
      val num_seqs = 5
      val rows = dut.p.mRows
      val cols = dut.p.mCols

      // number of bits for the input data
      val in_len = dut.p.inPrecision

      for(i <- 1 to num_seqs) {
        val neg = r.nextBoolean()
        val a = RosettaTestHelpers.randomIntMatrix(rows,cols,in_len, neg)
        println("Matrix a")
        printMatrix(a)
        poke(dut.io.out.ready, false)
        for(i <- 0 until rows)
          for( j <- 0 until cols)
            poke(dut.io.input(i)(j), scala.math.BigInt.apply(a(i)(j)))
        step(1)
        poke(dut.io.start, true)
        for(k <- 0  until in_len) {
          for (i <- 0 until rows)
            for (j <- 0 until cols)
              expect(dut.io.out.bits(i)(j), RosettaTestHelpers.extractBitPos(a(i)(j),k,in_len))
          step(1)
        }
        poke(dut.io.out.ready, true)
        poke(dut.io.start, false)
        step(1)
      }
    }

    // Chisel arguments to pass to chiselMainTest
    def testArgs = RosettaTestHelpers.stdArgs

    for{
      inPrecision <- 16 to 16
      rows <- 8 to 8
      cols <- 8 to 8

    } {
      // function that instantiates the Module to be tested
      val p = new SerializerUnitParams(inPrecision = inPrecision, mRows = rows, mCols = cols)
      def testModuleInstFxn = () => { Module(new SerializerUnit(p)) }
      // function that instantiates the Tester to test the Module
      def testTesterInstFxn = (dut: SerializerUnit) => new SerializerUnitTester(dut)

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
