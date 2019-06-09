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
import fpgatidbits.PlatformWrapper._

// Tester-derived class to give stimulus and observe the outputs for the
// Module to be tested
class BlockStridedRqGenTester(c: BlockStridedRqGen) extends Tester(c) {
  val r = scala.util.Random
  val base_offs: Int = 0x1000
  val nblocks: Int = 2
  val block_offs: Int = 0x100
  val intra_step: Int = 1
  val intra_burst: Int = 8
  val block_size: Int = 20

  poke(c.io.block_intra_step, intra_step)
  poke(c.io.block_intra_count, block_size)

  poke(c.io.in.bits.base, base_offs)
  poke(c.io.in.bits.block_step, block_offs)
  poke(c.io.in.bits.block_count, nblocks)

  poke(c.io.in.valid, 1)
  step(1)
  poke(c.io.in.valid, 0)

  poke(c.io.out.ready, 1)

  var bytes_left_in_block: Int = 0
  for (b ← 0 until nblocks) {
    bytes_left_in_block = block_size
    while(bytes_left_in_block > 0) {
      while (peek(c.io.out.valid) != 1) { step(1) }
      expect(c.io.out.bits.addr, base_offs + block_offs * b + (block_size - bytes_left_in_block))
      if(bytes_left_in_block >= intra_burst) {
        expect(c.io.out.bits.numBytes, intra_burst)
        bytes_left_in_block -= intra_burst
      } else {
        expect(c.io.out.bits.numBytes, intra_step)
        bytes_left_in_block -= intra_step
      }
      expect(c.io.out.bits.isWrite, 1)
      step(1)
    }
  }
}

class TestBlockStridedRqGen extends JUnitSuite {
  @Test def BlockStridedRqGenModuleTest {
    // Chisel arguments to pass to chiselMainTest
    def testArgs = BISMOTestHelpers.stdArgs
    // function that instantiates the Module to be tested
    def testModuleInstFxn = () ⇒ {
      Module(new BlockStridedRqGen(
        PYNQZ1Params.toMemReqParams(), true, 0
      ))
    }
    // function that instantiates the Tester to test the Module
    def testTesterInstFxn = (c: BlockStridedRqGen) ⇒ new BlockStridedRqGenTester(c)

    // actually run the test
    chiselMainTest(
      testArgs,
      testModuleInstFxn
    ) {
        testTesterInstFxn
      }
  }
}
