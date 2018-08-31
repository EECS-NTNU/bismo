// Author:  Davide Conficconi
// Date: 21/08/2018
// Revision: 0
import Chisel._
import bismo._
import fpgatidbits.ocm._
import org.scalatest.junit.JUnitSuite
import org.junit.Test
import RosettaTestHelpers._
class TestThresholdingUnit extends JUnitSuite {
  @Test def ThresholdingUnitTest {
    // Tester-derived class to give stimulus and observe the outputs for the
    // Module to be tested
    class ThresholdingUnitTester(dut: ThresholdingUnit) extends Tester(dut) {
      val r = scala.util.Random
      // number of re-runs for each test
      val num_seqs = 3
      //Input a single random matrix and then quantize
      // as thresholding unit with random thresholds is enough

      // number of bits for the input matrix
      val in_len = dut.p.inputBitPrecision
      //  number of bits for the output matrix
      val out_len = dut.p.maxOutputBitPrecision
      // spatial dimensions of the array
      val m = dut.p.matrixRows
      val n = dut.p.matrixColumns
      val unroll_rows = dut.p.unrollingFactorRows
      val unroll_factor = dut.p.unrollingFactorOutputPrecision
      val unroll_cols = dut.p.unrollingFactorColumns
      val mem_depth = dut.p.thresholdMemDepth


      // helper fuctions for more concise tests
      /*// wait up to <latency> cycles with valid=0 to create pipeline bubbles
      def randomNoValidWait(max: Int = latency) = {
        val numNoValidCycles = r.nextInt(max+1)
        poke(dut.io.valid, 0)
        step(numNoValidCycles)
      }*/
      def printMatrix(a: Seq[Seq[Int]]) = {
          for(i <- 0 to a.size-1){
          for(j <- 0 to a(i).size-1){
            print(a(i)(j))
            print(" ")   
          }
          println(" ")     
        }
      }

      for(i <- 1 to num_seqs) {

        val a = RosettaTestHelpers.randomIntMatrix(1, n, in_len, false)
        println("Matrix a")
        printMatrix(a)
        val th = RosettaTestHelpers.randomIntMatrix(1, (scala.math.pow(2,out_len)toInt), in_len, false)
        println("matrrix th")
        printMatrix(th)
        val golden = RosettaTestHelpers.quantizeMatrix(a, th)
        println("Matrix GOld")
        printMatrix(golden)
        for(i<-0 to a.head.size-1) {
          poke(dut.io.inputMatrix.bits.i(0)(i), scala.math.BigInt.apply(a(0)(i)) )
        }
        poke(dut.io.inputMatrix.valid, true)
        step(1)
        step(1)
        for(i<-0 to a(0).size-1){
          expect(dut.io.outputMatrix.bits.o(0)(i), scala.math.BigInt.apply(golden(0)(i)) )

        }
        poke(dut.io.inputMatrix.valid, false)
        step(1)
        poke(dut.io.inputMatrix.bits.i(0)(i), scala.math.BigInt.apply(0) )
        step(1)

        }
      
    }

    // Chisel arguments to pass to chiselMainTest
    def testArgs = RosettaTestHelpers.stdArgs

    for{
      inPrecision <- 4 to 4
      maxOutPrecision <- 2 to 2
      rowsDM <- 1 to 1
      columnsDN <- 8 to 8
      thDepth <- 8 to 8 // MY WORRIES: should it be equal to the row of the matrix?
      unrollingBB <- 4 to 4
      unrollingRows <- 1 to 1
      unrollingCols <- 8 to 8
    } {
      // function that instantiates the Module to be tested
      val p = new ThresholdingUnitParams(inPrecision, maxOutPrecision, rowsDM , columnsDN, thDepth, unrollingBB, unrollingRows, unrollingCols)
      def testModuleInstFxn = () => { Module(new ThresholdingUnit(p)) }
      // function that instantiates the Tester to test the Module
      def testTesterInstFxn = (dut: ThresholdingUnit) => new ThresholdingUnitTester(dut)

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
