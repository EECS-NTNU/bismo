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

class TestP2SKernel() extends JUnitSuite {
  @Test def P2SKernelTest {
    // Tester-derived class to give stimulus and observe the outputs for the
    // Module to be tested
    class P2SKernelTester(dut: P2SKernel
    ) extends Tester(dut) {
      val r = scala.util.Random
      // number of re-runs for each test
      val num_seqs = 1


      for (i <- 1 to num_seqs) {

    }

    // Chisel arguments to pass to chiselMainTest
    def testArgs = RosettaTestHelpers.stdArgs

    for {
      inbw <- 8 to 8
      elemXWord <- 8 to 8
      outSize <- 64

    } {
      // function that instantiates the Module to be tested
      val p = new P2SKernelParams(
        maxInBw = inbw, nInElemPerWord = elemXWord, outStreamSize = outSize
      )

      def testModuleInstFxn = () => {
        Module(new P2SKernel(p))
      }

      // function that instantiates the Tester to test the Module
      def testTesterInstFxn = (dut: P2SKernel)  => new P2SKernelTester (dut)

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
