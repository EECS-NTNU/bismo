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
import org.scalatest.junit.JUnitSuite
import org.junit.Test
import BISMOTestHelpers._
import fpgatidbits.streams._

// Tester-derived class to give stimulus and observe the outputs for the
// Module to be tested
class BlockSeqGenTester(c: BlockSequenceGenerator) extends Tester(c) {
  val r = scala.util.Random
  var g_ptr: Int = 100
  var g_elems_left: Int = 10
  val g_blocksize: Int = 3

  poke(c.io.out.ready, 0)
  poke(c.io.cmd.bits.start, g_ptr)
  poke(c.io.cmd.bits.count, g_elems_left)
  poke(c.io.cmd.bits.blockSize, g_blocksize)
  poke(c.io.cmd.valid, 1)
  step(1)
  poke(c.io.cmd.valid, 0)

  while(g_elems_left > 0) {
    val exp_bsize = if(g_elems_left < g_blocksize) g_elems_left else g_blocksize
    step(1)
    while(peek(c.io.out.valid) == 0) { step(1) }
    poke(c.io.out.ready, 1)
    expect(c.io.out.bits.start, g_ptr)
    expect(c.io.out.bits.count, exp_bsize)
    g_ptr = g_ptr + exp_bsize
    g_elems_left = g_elems_left - exp_bsize
  }
}

class TestBlockSeqGen extends JUnitSuite {
  @Test def BlockSeqGenModuleTest {
    // Chisel arguments to pass to chiselMainTest
    def testArgs = BISMOTestHelpers.stdArgs
    // function that instantiates the Module to be tested
    def testModuleInstFxn = () => { Module(new BlockSequenceGenerator(
      32
    )) }
    // function that instantiates the Tester to test the Module
    def testTesterInstFxn = (c: BlockSequenceGenerator) => new BlockSeqGenTester(c)

    // actually run the test
    chiselMainTest(
      testArgs,
      testModuleInstFxn
    ) {
      testTesterInstFxn
    }
  }
}
