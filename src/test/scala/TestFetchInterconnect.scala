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
import fpgatidbits.PlatformWrapper._

// Tester-derived class to give stimulus and observe the outputs for the
// Module to be tested
class FetchInterconnectTester(c: FetchInterconnect) extends Tester(c) {
  val r = scala.util.Random
  // number of writes to test
  val num_writes = 100
  // number of instantiated nodes
  val num_nodes = c.myP.numNodes
  // bitwidth of packet data
  val dbits = c.myP.mrp.dataWidth
  // maximum address inside BRAM
  val maxaddr = (1 << c.myP.numAddrBits) - 1

  def randomNoValidWait(max: Int = num_nodes + 1) = {
    val numNoValidCycles = r.nextInt(max + 1)
    poke(c.io.in.valid, 0)
    for (cyc ← 0 until numNoValidCycles) {
      step(1)
      // all write enables should be zero
      c.io.node_out.map(x ⇒ expect(x.writeEn, 0))
    }
  }

  def checkNodeMemStatus(
    golden_valid: Seq[Int],
    golden_id: Seq[Int],
    golden_addr: Seq[Int],
    golden_data: Seq[BigInt]) =
    {
      for (n ← 0 until num_nodes) {
        if (golden_valid(n) == 1 && n == golden_id(n)) {
          expect(c.io.node_out(n).writeEn, 1)
          expect(c.io.node_out(n).addr, golden_addr(n))
          expect(c.io.node_out(n).writeData, golden_data(n))
        } else {
          expect(c.io.node_out(n).writeEn, 0)
        }
      }
    }

  randomNoValidWait()
  var exp_valid = (1 to num_nodes).map(x ⇒ 0).toList
  var exp_id = (1 to num_nodes).map(x ⇒ 0).toList
  var exp_addr = (1 to num_nodes).map(x ⇒ 0).toList
  var exp_data = (1 to num_nodes).map(x ⇒ BigInt(0)).toList

  for (i ← 1 to num_writes) {
    val test_data = BISMOTestHelpers.randomIntVector(dbits, 1, false)
    val test_data_bigint = scala.math.BigInt.apply(test_data.mkString, 2)
    val test_id = r.nextInt(num_nodes + 1)
    val test_addr = r.nextInt(maxaddr + 1)
    poke(c.io.in.bits.data, test_data_bigint)
    poke(c.io.in.bits.id, test_id)
    poke(c.io.in.bits.addr, test_addr)
    poke(c.io.in.valid, 1)
    // update expected state
    exp_valid = List(1) ++ exp_valid.dropRight(1)
    exp_id = List(test_id) ++ exp_id.dropRight(1)
    exp_addr = List(test_addr) ++ exp_addr.dropRight(1)
    exp_data = List(test_data_bigint) ++ exp_data.dropRight(1)
    step(1)
    poke(c.io.in.valid, 0)
    for (ws ← 0 to test_id) {
      checkNodeMemStatus(exp_valid, exp_id, exp_addr, exp_data)
      exp_valid = List(0) ++ exp_valid.dropRight(1)
      exp_id = List(test_id) ++ exp_id.dropRight(1)
      exp_addr = List(test_addr) ++ exp_addr.dropRight(1)
      exp_data = List(test_data_bigint) ++ exp_data.dropRight(1)
      step(1)
    }
  }
}

class TestFetchInterconnect extends JUnitSuite {
  @Test def FetchInterconnectTest {
    // Chisel arguments to pass to chiselMainTest
    def testArgs = BISMOTestHelpers.stdArgs
    // function that instantiates the Module to be tested
    def testModuleInstFxn = () => { Module(new FetchInterconnect(
      new FetchStageParams(
        numLHSMems = 5, numRHSMems = 5,
        numAddrBits = 10, mrp = PYNQZ1Params.toMemReqParams(), thrEntriesPerMem = 8
      )
    )) }
    // function that instantiates the Tester to test the Module
    def testTesterInstFxn = (c: FetchInterconnect) ⇒ new FetchInterconnectTester(c)

    // actually run the test
    chiselMainTest(
      testArgs,
      testModuleInstFxn
    ) {
        testTesterInstFxn
      }
  }
}
