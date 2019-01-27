// Author:  Davide Conficconi
// Date: 23/08/2018
// Revision: 0
import Chisel._
import bismo._

import org.scalatest.junit.JUnitSuite
import org.junit.Test
import RosettaTestHelpers._
class TestThresholdingBuildingBlock extends JUnitSuite {
  @Test def ThresholdingBuildingBlockTest {
    // Tester-derived class to give stimulus and observe the outputs for the
    // Module to be tested
    class ThresholdingBuildingBlockTester(dut: ThresholdingBuildingBlock) extends Tester(dut) {
      val r = scala.util.Random
      // number of re-runs for each test
      val num_seqs = 100

      // number of bits for the input data
      val in_len = dut.p.inPrecision
      //  number of bits for the output data
      val out_len = dut.p.outPrecision
      val unroll_factor = dut.p.popcountUnroll
      val rows = 8
      val cols = 8
      val inNeg = false
      val thNeg = false
      //TODO: there is a problem with negative numbers and 2c2 representation

      for (i ← 1 to num_seqs) {

        val a = RosettaTestHelpers.randomIntMatrix(rows, cols, in_len, inNeg)
        println("Matrix a")
        printMatrix(a)

        val th = RosettaTestHelpers.randomIntMatrix(rows, scala.math.pow(2, out_len).toInt - 1, in_len, thNeg)
        println("matrrix th")
        printMatrix(th)
        val golden = RosettaTestHelpers.quantizeMatrix(a, th)

        println("Matrix GOld")
        printMatrix(golden)
        //feed inputs
        for (i ← 0 until rows) {
          for (curr_col ← 0 until cols) {
            for (j ← 0 until unroll_factor) {
              poke(dut.io.inVector(j), scala.math.BigInt.apply(a(i)(curr_col)))
              poke(dut.io.thVector(j), scala.math.BigInt.apply(th(i)(j)))
              println("Row:" + i + " Col:" + curr_col + "Threshold: " + j)
              //println(scala.math.BigInt.apply(a(i)(curr_col)).toString(2) + " " + scala.math.BigInt.apply(th(i)(j)).toString(2))
            }
            step(1)

            //expect right comparison and clear on every cycle
            println("Golden value: " + golden(i)(curr_col))
            expect(dut.io.outValue, golden(i)(curr_col))
            poke(dut.io.clearAcc, true)
          }

        }

      }
    }

    // Chisel arguments to pass to chiselMainTest
    def testArgs = RosettaTestHelpers.stdArgs

    for {
      inPrecision ← 16 to 16
      outPrecision ← 2 to 2
      unrolling ← 3 to 3
    } {
      // function that instantiates the Module to be tested
      val p = new ThresholdingBuildingBlockParams(inPrecision, unrolling, outPrecision)
      def testModuleInstFxn = () ⇒ { Module(new ThresholdingBuildingBlock(p)) }
      // function that instantiates the Tester to test the Module
      def testTesterInstFxn = (dut: ThresholdingBuildingBlock) ⇒ new ThresholdingBuildingBlockTester(dut)

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
