// Copyright (c) 2018 Norwegian University of Science and Technology (NTNU)
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
// * Neither the name of [project] nor the names of its
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
import RosettaTestHelpers._

class TestDotProductArray extends JUnitSuite {
  @Test def DotProdArrayTest {
    // Tester-derived class to give stimulus and observe the outputs for the
    // Module to be tested
    class DotProductArrayTester(c: DotProductArray) extends Tester(c) {
      val r = scala.util.Random
      // number of re-runs for each test
      val num_seqs = 100
      // number of bits in each operand
      val pc_len = c.p.dpuParams.pcParams.numInputBits
      // max shift steps for random input
      val max_shift = c.p.dpuParams.maxShiftSteps
      // spatial dimensions of the array
      val m = c.p.m
      val n = c.p.n
      // latency from inputs changed to accumulate update
      val latency = c.p.getLatency()

      // helper fuctions for more concise tests
      // wait up to <latency> cycles with valid=0 to create pipeline bubbles
      def randomNoValidWait(max: Int = latency) = {
        val numNoValidCycles = r.nextInt(max + 1)
        poke(c.io.valid, 0)
        step(numNoValidCycles)
      }

      for (i ← 1 to num_seqs) {
        // generate two random int matrices a[m_test][k_test] and b[n_test][k_test] s.t.
        // m_test % m = 0, n_test % n = 0, k_test % pc_len = 0
        val seq_len = 1 + r.nextInt(17)
        val k_test = pc_len * seq_len
        // TODO add more m and n tiles, clear accumulator in between
        val m_test = m
        val n_test = n
        // precision in bits, each between 1 and max_shift/2 bits
        // such that their sum won't be greater than max_shift
        val precA = 1 + r.nextInt(max_shift / 2)
        val precB = 1 + r.nextInt(max_shift / 2)
        assert(precA + precB <= max_shift)
        // produce random binary test vectors and golden result
        val negA = r.nextBoolean
        val negB = r.nextBoolean
        val a = RosettaTestHelpers.randomIntMatrix(m_test, k_test, precA, negA)
        val b = RosettaTestHelpers.randomIntMatrix(m_test, k_test, precB, negB)
        val golden = RosettaTestHelpers.matrixProduct(a, b)
        // iterate over each combination of bit positions for bit serial
        for (bitA ← 0 to precA - 1) {
          val negbitA = negA & (bitA == precA - 1)
          for (bitB ← 0 to precB - 1) {
            // enable negation if combination of bit positions is negative
            val negbitB = negB & (bitB == precB - 1)
            val doNeg = if (negbitA ^ negbitB) 1 else 0
            poke(c.io.negate, doNeg)
            // shift is equal to sum of current bit positions
            poke(c.io.shiftAmount, bitA + bitB)
            for (j ← 0 to seq_len - 1) {
              // set clear bit only on the very first iteration
              val doClear = if (j == 0 & bitA == 0 & bitB == 0) 1 else 0
              poke(c.io.clear_acc, doClear)
              // insert stimulus for left-hand-side matrix tile
              for (i_m ← 0 to m - 1) {
                val seqA_bs = RosettaTestHelpers.intVectorToBitSerial(a(i_m), precA)
                val curA = seqA_bs(bitA).slice(j * pc_len, (j + 1) * pc_len)
                poke(c.io.a(i_m), scala.math.BigInt.apply(curA.mkString, 2))
              }
              // insert stimulus for right-hand-side matrix tile
              for (i_n ← 0 to n - 1) {
                val seqB_bs = RosettaTestHelpers.intVectorToBitSerial(b(i_n), precB)
                val curB = seqB_bs(bitB).slice(j * pc_len, (j + 1) * pc_len)
                poke(c.io.b(i_n), scala.math.BigInt.apply(curB.mkString, 2))
              }
              poke(c.io.valid, 1)
              step(1)
              // emulate random pipeline bubbles
              randomNoValidWait()
            }
          }
        }
        // remove valid input in next cycle
        poke(c.io.valid, 0)
        // wait until all inputs are processed
        step(latency - 1)
        // check produced matrix against golden result
        for (i_m ← 0 to m - 1) {
          for (i_n ← 0 to n - 1) {
            expect(c.io.out(i_m)(i_n), golden(i_m)(i_n))
          }
        }
      }
    }

    // Chisel arguments to pass to chiselMainTest
    def testArgs = RosettaTestHelpers.stdArgs
    // function that instantiates the Module to be tested
    val pPC = new PopCountUnitParams(36)
    val pDP = new DotProductUnitParams(pPC, 32, 16)
    val p = new DotProductArrayParams(pDP, 4, 4, 0)
    def testModuleInstFxn = () ⇒ { Module(new DotProductArray(p)) }
    // function that instantiates the Tester to test the Module
    def testTesterInstFxn = (c: DotProductArray) ⇒ new DotProductArrayTester(c)

    // actually run the test
    chiselMainTest(
      testArgs,
      testModuleInstFxn
    ) {
        testTesterInstFxn
      }
  }
}
