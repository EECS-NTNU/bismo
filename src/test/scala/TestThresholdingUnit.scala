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
      val num_seqs = 1
      //Input a single random matrix and then quantize
      // as thresholding unit with random thresholds is enough

      // number of bits for the input matrix
      val in_len = dut.p.inputBitPrecision
      //  number of bits for the output matrix
      val out_len = dut.p.maxOutputBitPrecision
      // spatial dimensions of the array
      val m = dut.p.matrixRows
      val n = dut.p.matrixColumns
      val unroll_factor = dut.p.unrollingFactor

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
        // generate two random int matrices a[m_test][k_test] and b[n_test][k_test] s.t.
        // m_test % m = 0, n_test % n = 0, k_test % pc_len = 0
        /*val seq_len = 1 + r.nextInt(17)
        val k_test = pc_len * seq_len
        // TODO add more m and n tiles, clear accumulator in between
        val m_test = m
        val n_test = n
        // precision in bits, each between 1 and max_shift/2 bits
        // such that their sum won't be greater than max_shift
        val precA = 1 + r.nextInt(max_shift/2)
        val precB = 1 + r.nextInt(max_shift/2)
        assert(precA + precB <= max_shift)
        // produce random binary test vectors and golden result
        val negA = r.nextBoolean
        val negB = r.nextBoolean*/
        val a = RosettaTestHelpers.randomIntMatrix(m, n, in_len, false)
        println("Matrix a")
        printMatrix(a)
        val th = RosettaTestHelpers.randomIntMatrix(m, out_len, out_len, false)
        println("matrrix th")
        printMatrix(th)
        val golden = RosettaTestHelpers.quantizeMatrix(a, th)
        println("Matrix GOld")
        printMatrix(golden)
        poke(dut.io.inputMatrix.i, a)
        poke(dut.io.inputMatrix.iValid, true)
        for(i <- 0 to m){
          expect(dut.io.thInterf.thresholdRequest.addr, i)
          step(1)
          poke(dut.io.thInterf.thresholdResponse.readData, th(i))
          expect(dut.io.outputMatrix.oValid, false)
        }
        step(1)
        expect(dut.io.outputMatrix.oValid, true)
        for(i_m <- 0 to m-1) {
          for(i_n <- 0 to n-1) {
            expect(dut.io.outputMatrix.o(i_m)(i_n), golden(i_m)(i_n))
          }
        }

        // iterate over each combination of bit positions for bit serial
        /*for(bitA <- 0 to precA-1) {
          val negbitA = negA & (bitA == precA-1)
          for(bitB <- 0 to precB-1) {
            // enable negation if combination of bit positions is negative
            val negbitB = negB & (bitB == precB-1)
            val doNeg = if(negbitA ^ negbitB) 1 else 0
            poke(dut.io.negate, doNeg)
            // shift is equal to sum of current bit positions
            poke(dut.io.shiftAmount, bitA+bitB)
            for(j <- 0 to seq_len-1) {
              // set clear bit only on the very first iteration
              val doClear = if(j == 0 & bitA == 0 & bitB == 0) 1 else 0
              poke(dut.io.clear_acc, doClear)
              // insert stimulus for left-hand-side matrix tile
              for(i_m <- 0 to m-1) {
                val seqA_bs = RosettaTestHelpers.intVectorToBitSerial(a(i_m), precA)
                val curA = seqA_bs(bitA).slice(j*pc_len, (j+1)*pc_len)
                poke(dut.io.a(i_m), scala.math.BigInt.apply(curA.mkString, 2))
              }
              // insert stimulus for right-hand-side matrix tile
              for(i_n <- 0 to n-1) {
                val seqB_bs = RosettaTestHelpers.intVectorToBitSerial(b(i_n), precB)
                val curB = seqB_bs(bitB).slice(j*pc_len, (j+1)*pc_len)
                poke(dut.io.b(i_n), scala.math.BigInt.apply(curB.mkString, 2))
              }
              poke(dut.io.valid, 1)
              step(1)
              // emulate random pipeline bubbles
              randomNoValidWait()
            }
          }
        }
        // remove valid input in next cycle
        poke(dut.io.valid, 0)
        // wait until all inputs are processed
        step(latency-1)
        // check produced matrix against golden result
        for(i_m <- 0 to m-1) {
          for(i_n <- 0 to n-1) {
            expect(dut.io.out(i_m)(i_n), golden(i_m)(i_n))
          }
        }*/
      }
    }

    // Chisel arguments to pass to chiselMainTest
    def testArgs = RosettaTestHelpers.stdArgs

    for{
      inPrecision <- 32 to 32
      maxOutPrecision <- 4 to 4
      rowsDM <- 8 to 8
      columnsDN <- 8 to 8
      thDepth <- 8 to 8 // MY WORRIES: should it be equal to the row of the matrix?
      thWidth <- 32 to 32 // I think it is the numbr of columns times the maximum out precision
      unrolling <- 0 to 0
      //popc_width <- for(b <- 4 to 10) yield 1 << b
    } {
      // function that instantiates the Module to be tested
      val p = new ThresholdingUnitParams(inPrecision, maxOutPrecision, rowsDM , columnsDN, thDepth, thWidth, unrolling)
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
