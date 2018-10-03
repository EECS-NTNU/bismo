// Author:  Davide Conficconi
// Date: 21/08/2018
// Revision: 0

//THIS FILE IS INTENDED JUST FOR WAVEFORM VISUALIZATION
import Chisel._
import bismo._
import fpgatidbits.ocm._
import org.scalatest.junit.JUnitSuite
import org.junit.Test
import RosettaTestHelpers._
import fpgatidbits.PlatformWrapper.TesterWrapperParams

class TestEmuTestThrStage extends JUnitSuite {
  @Test def EmuTestThrStageTest {
    // Tester-derived class to give stimulus and observe the outputs for the
    // Module to be tested
    class EmuTestThrStageTester(dut: EmuTestThrStage) extends Tester(dut) {
      val r = scala.util.Random
      // number of re-runs for each test
      val num_seqs = 1
      //Input a single random matrix and then quantize
      // as thresholding unit with random thresholds is enough

      // number of bits for the input matrix
      val in_len = dut.myP.getInBits()
      //  number of bits for the output matrix
      val out_len = dut.myP.thuParams.maxOutputBitPrecision
      val thNumber = 1//dut.myP.maxThresholdNumber
      // spatial dimensions of the array
      val m = dut.myP.getRows()
      val n = dut.myP.getCols()
      val unroll_rows = dut.myP.getUnrollRows()
      val unroll_factor = dut.myP.getThUnroll()
      val unroll_cols = dut.myP.getUnrollCols()
      val mem_depth = dut.myP.thresholdMemDepth
      val negVal = false
      val negTh = false
      var iterFactor = 0

      for(i <- 1 to num_seqs) {

        val a = Seq(Seq(1,2,3),Seq(4,5,6))//RosettaTestHelpers.randomIntMatrix(m, n, in_len, negVal)
        println("Matrix a")
        printMatrix(a)
        val th = Seq(Seq(1),Seq(0))//RosettaTestHelpers.randomIntMatrix(m, thNumber, in_len, negTh)
        println("matrrix th")
        printMatrix(th)
        val golden = RosettaTestHelpers.quantizeMatrix(a, th)
        println("Matrix GOld")


        printMatrix(golden)

        //var tmp_act : BigInt = 0
        //Control part
          // care on this that change results on test
          poke(dut.io.ctrl.runTimeThrNumber, scala.math.BigInt.apply(1))
          poke(dut.io.ctrl.actOffset, scala.math.BigInt.apply(0))
          poke(dut.io.ctrl.thrOffset, scala.math.BigInt.apply(0))
          poke(dut.io.ctrl.writeEn, scala.math.BigInt.apply(1))
          poke(dut.io.ctrl.writeAddr, scala.math.BigInt.apply(0))
          //fill bram
          for(i <- 0 until m) {
            for (j <- 0 until n) {
              //tmp_act = tmp_act | (scala.math.BigInt.apply(a(i)(j)) << (j * dut.myP.getInBits()))
              //println("[TESTBENCH] " + tmp_act)
              poke(dut.io.inMemory_act_data, scala.math.BigInt.apply(a(i)(j)))
              poke(dut.io.inMemory_act_sel_r, scala.math.BigInt.apply(i))
              poke(dut.io.inMemory_act_sel_c, scala.math.BigInt.apply(j))
              poke(dut.io.inMemory_act_addr, scala.math.BigInt.apply(0))
              poke(dut.io.inMemory_act_write, scala.math.BigInt.apply(1))
              step(1)
            }

            //tmp_act = 0
          }

        //var tmp_th : BigInt = 0
          for(i <- 0 until m) {
            for (j <- 0 until unroll_factor) {
              //tmp_th = tmp_th | (scala.math.BigInt.apply(th(i)(j)) << (j * dut.myP.getInBits()))
              //println("[TESTBENCH] " + tmp_th)
              if(j < thNumber){
                poke(dut.io.inMemory_thr_sel_r, scala.math.BigInt.apply(i))
                poke(dut.io.inMemory_thr_sel_c, scala.math.BigInt.apply(j))
                poke(dut.io.inMemory_thr_addr, scala.math.BigInt.apply(0))
                poke(dut.io.inMemory_thr_data, scala.math.BigInt.apply(th(i)(j)) )
                poke(dut.io.inMemory_thr_write, scala.math.BigInt.apply(1))
              } else {
                poke(dut.io.inMemory_thr_sel_r, scala.math.BigInt.apply(i))
                poke(dut.io.inMemory_thr_sel_c, scala.math.BigInt.apply(j))
                poke(dut.io.inMemory_thr_addr, scala.math.BigInt.apply(0))
                poke(dut.io.inMemory_thr_data, scala.math.BigInt.apply(0))
                poke(dut.io.inMemory_thr_write, scala.math.BigInt.apply(1))
              }

              step(1)
            }


            //tmp_th = 0
          }

        step(1)
        poke(dut.io.start, scala.math.BigInt.apply(1))
        step(dut.myP.thuParams.getLatency())
        step(4)
        poke(dut.io.start, scala.math.BigInt.apply(0))
        step(1)
        //TODO Check results
        for(i <- 0 until dut.myP.getRows())
          for(j <- 0 until dut.myP.getCols()) {
            poke(dut.io.resmem_addr_e, scala.math.BigInt.apply(0))
            poke(dut.io.resmem_addr_r,scala.math.BigInt.apply(i))
            poke(dut.io.resmem_addr_c, scala.math.BigInt.apply(j))
            step(1)
            expect(dut.io.resmem_data, golden(i)(j))
            step(1)
          }
      }

    }

    // Chisel arguments to pass to chiselMainTest
    def testArgs = RosettaTestHelpers.stdArgs

    for{
      rows <- 2 to 2
      cols <- 3 to 3
      inBit <- 32 to 32
      outBit <- 2 to 2
      unroll <- 3 to 3
    } {
      val emuP = TesterWrapperParams
      def testModuleInstFxn = () => { Module(new EmuTestThrStage(rows,cols,inBit,outBit, unroll, emuP)) }
      // function that instantiates the Tester to test the Module
      def testTesterInstFxn = (dut: EmuTestThrStage) => new EmuTestThrStageTester(dut)

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
