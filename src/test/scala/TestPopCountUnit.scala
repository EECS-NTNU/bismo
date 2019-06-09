// Copyright (c) 2018 Norwegian University of Science and Technology (NTNU)
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

// Tester-derived class to give stimulus and observe the outputs for the
// Module to be tested
class PopCountUnitTester(c: PopCountUnit) extends Tester(c) {
  val r = scala.util.Random
  // number of sequences to test
  val num_seqs = 10
  // length of each sequence in bits, equal to popcount width
  val seq_len = c.p.numInputBits
  // latency in cycles from input updated to result updated
  val latency = c.p.getLatency()

  def cleanTest(test_seq_str: String, golden: Int) = {
    // clear the pipeline by putting in zeros
    poke(c.io.in, scala.math.BigInt.apply("0" * seq_len, 2))
    step(1)
    poke(c.io.in, scala.math.BigInt.apply(test_seq_str, 2))
    step(1)
    poke(c.io.in, scala.math.BigInt.apply("0" * seq_len, 2))
    for (i ← 0 until latency - 2) {
      step(1)
      peek(c.io.out)
    }
    //step(latency-2)
    expect(c.io.out, 0)
    step(1)
    expect(c.io.out, golden)
    step(1)
    expect(c.io.out, 0)
  }

  // test all-zeroes and all-ones
  cleanTest("0" * seq_len, 0)
  cleanTest("1" * seq_len, seq_len)

  for (seq_cnq ← 1 to num_seqs) {
    val test_seq = BISMOTestHelpers.randomIntVector(seq_len, 1, false)
    cleanTest(test_seq.mkString, test_seq.reduce(_ + _))
  }
}

class TestPopCountUnit extends JUnitSuite {
  @Test def PopCountModuleTest {
    for (b ← 4 to 10) {
      // Chisel arguments to pass to chiselMainTest
      def testArgs = BISMOTestHelpers.stdArgs
      // function that instantiates the Module to be tested
      def testModuleInstFxn = () ⇒ {
        Module(new PopCountUnit(
          new PopCountUnitParams(numInputBits = 1 << b)
        ))
      }
      // function that instantiates the Tester to test the Module
      def testTesterInstFxn = (c: PopCountUnit) ⇒ new PopCountUnitTester(c)

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
