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
      val num_seqs = 3

      // number of bits for the input data
      val in_len = dut.p.inPrecision
      //  number of bits for the output data
      val out_len = dut.p.outPrecision
      val unroll_factor = dut.p.popcountUnroll
      val rows = 1
      val cols = 1

      for(i <- 1 to num_seqs) {

        val a = List(List(1, 2, 3))//RosettaTestHelpers.randomIntMatrix(rows, cols,  in_len, false)
        println("Matrix a")
        printMatrix(a)

        val th = List(List(0,1))//RosettaTestHelpers.randomIntMatrix(rows, scala.math.pow(2,out_len)toInt, in_len, false)
        println("matrrix th")
        printMatrix(th)
        //println(th)java
        val golden = RosettaTestHelpers.quantizeMatrix(a, th)
        println("Matrix GOld")
        printMatrix(golden)

        for(i <- 0 to rows - 1){
          //val seqA_bs = RosettaTestHelpers.intVectorToBitSerial(a(i), in_len)
          //val seqTh_bs = RosettaTestHelpers.intVectorToBitSerial(th(i), in_len)
          //println(seqA_bs)
          //println(seqTh_bs)
          // Input the same number and different thresholds per an unroll factor times
          // quantize a number
          for(curr_col <- 0 to cols - 1) {
            for (j <- 0 to unroll_factor - 1) {
              poke(dut.io.inVector(j), scala.math.BigInt.apply(a(i)(curr_col)))
              poke(dut.io.thVector(j), scala.math.BigInt.apply(th(i)(j)))
            }
            step(1)
            //println(dut.io.outValue)
            expect(dut.io.outValue, golden(i)(curr_col))
            
          }
          step(1)
        }

        //poke(dut.io.inVector, scala.math.BigInt.apply(a.mkString, 2))
        //poke(dut.io.thVector, scala.math.BigInt.apply(th.mkString,2))
        //step(1)
        //expect(dut.io.outValue == golden)


      }
    }

    // Chisel arguments to pass to chiselMainTest
    def testArgs = RosettaTestHelpers.stdArgs

    for{
      inPrecision <- 32 to 32
      outPrecision <- 1 to 1
      unrolling <- 2 to 2
      //popc_width <- for(b <- 4 to 10) yield 1 << b
    } {
      // function that instantiates the Module to be tested
      val p = new ThresholdingBuildingBlockParams(inPrecision, unrolling, outPrecision)
      def testModuleInstFxn = () => { Module(new ThresholdingBuildingBlock(p)) }
      // function that instantiates the Tester to test the Module
      def testTesterInstFxn = (dut: ThresholdingBuildingBlock) => new ThresholdingBuildingBlockTester(dut)

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
