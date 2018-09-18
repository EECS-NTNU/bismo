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
      val num_seqs = 5
      //Input a single random matrix and then quantize
      // as thresholding unit with random thresholds is enough

      // number of bits for the input matrix
      val in_len = dut.p.inputBitPrecision
      //  number of bits for the output matrix
      val out_len = dut.p.maxOutputBitPrecision
      val thNumber = dut.p.thresholdNumber
      // spatial dimensions of the array
      val m = dut.p.matrixRows
      val n = dut.p.matrixColumns
      val unroll_rows = dut.p.unrollingFactorRows
      val unroll_factor = dut.p.unrollingFactorOutputPrecision
      val unroll_cols = dut.p.unrollingFactorColumns
      val mem_depth = dut.p.thresholdMemDepth
      val negVal = false
      val negTh = false
      var iterFactor = 0


      for(i <- 1 to num_seqs) {

        val a = RosettaTestHelpers.randomIntMatrix(m, n, in_len, negVal)
        println("Matrix a")
        printMatrix(a)
        val th = RosettaTestHelpers.randomIntMatrix(m, thNumber, in_len, negTh)
        println("matrrix th")
        printMatrix(th)
        val golden = RosettaTestHelpers.quantizeMatrix(a, th)
        println("Matrix GOld")



        printMatrix(golden)
        //TODO generalize in the clock cycle to wait
        for(i<-0 until m)
          for(j <- 0 until n)
            poke(dut.io.inputMatrix.bits.i(i)(j), scala.math.BigInt.apply(a(i)(j)) )
        poke(dut.io.inputMatrix.valid, true)
        step(1)
        poke(dut.io.inputMatrix.valid, false)
        for(steps <- 0 until dut.p.totLatency){
          for(i<- 0 until m)
            for(j<- 0 until unroll_factor) {
              poke(dut.io.thInterf.thresholdData(i)(j), scala.math.BigInt.apply(th(i)( (iterFactor + j) )))
            }
          step(1)
          println(iterFactor)
          iterFactor = iterFactor + 1
        }
        //step(1)
        //step(dut.p.thresholdLatency)
        //step(1)
        //step(1)
        for(i<-0 until m)
          for(j<- 0 until n)
          expect(dut.io.outputMatrix.bits.o(i)(j), scala.math.BigInt.apply(golden(i)(j)) )
        //expect(dut.io.outputMatrix.valid, true
        expect(dut.io.outputMatrix.valid, true)
        poke(dut.io.outputMatrix.ready, true)
        step(1)
        poke(dut.io.outputMatrix.ready, false)
        iterFactor = 0

      }
      
    }

    // Chisel arguments to pass to chiselMainTest
    def testArgs = RosettaTestHelpers.stdArgs

    for{
      inPrecision <- 4 to 4
      maxOutPrecision <- 1 to 1
      rowsDM <- 2 to 2
      columnsDN <- 2 to 2
      thDepth <- 8 to 8 // MY WORRIES: should it be equal to the row of the matrix?
      unrollingBB <- 1 to 1
      unrollingRows <- 2 to 2
      unrollingCols <- 2 to 2
    } {
      val thBBParams = new ThresholdingBuildingBlockParams(	inPrecision = inPrecision, popcountUnroll = unrollingBB,  outPrecision = maxOutPrecision)
      // function that instantiates the Module to be tested
      val p = new ThresholdingUnitParams(thBBParams, inputBitPrecision = inPrecision, maxOutPrecision, rowsDM , columnsDN, thDepth, unrollingBB, unrollingRows, unrollingCols)
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
