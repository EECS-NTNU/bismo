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

package bismo

import Chisel._
import fpgatidbits.dma._
import fpgatidbits.ocm._
import fpgatidbits.streams._
import fpgatidbits.PlatformWrapper._
import fpgatidbits.synthutils.PrintableParam

// ExecStage is one of thre three components of the BISMO pipeline, which
// contains infrastructure to concurrently fetch data from DRAM into BRAMs while
// the other stages are doing their thing.

class FetchStageParams(
  val numLHSMems: Int, // number of LHS memories
  val numRHSMems: Int, // number of RHS memories
  val numAddrBits: Int, // number of bits for address inside target memory
  val mrp: MemReqParams, // memory request params for platform
  val bramWrLat: Int = 1, // number of cycles until data written to BRAM
  val thrEntriesPerMem: Int = 8,
  val thsCols: Int = 3) extends PrintableParam {
  def headersAsList(): List[String] = {
    return List("nodes", "rd_width")
  }

  def contentAsList(): List[String] = {
    return List(numNodes, mrp.dataWidth).map(_.toString)
  }

  // total number of nodes (LHS or RHS mems) targeted by the FetchStage + Thresholds
  val numNodes = numLHSMems + numRHSMems //+ numLHSMems + thsCols
  // number of ID bits to identify a node
  def getIDBits(): Int = {
    return log2Up(getNumNodes())
  }

  // total number of bits for identifying packet target
  def getTotalRouteBits(): Int = {
    return getIDBits() + numAddrBits
  }

  // number of nodes
  def getNumNodes(): Int = {
    return numNodes
  }

  // number of max cycles between when data is emitted from the DMA
  // engine (StreamReader) until it is written into its target BRAM
  def getDMAtoBRAMLatency(): Int = {
    // 1 cycle per hop in the interconnect
    val max_interconnect_cycles = getNumNodes()
    val routegen_cycles = 1 // due to queuing between routegen and interconnect
    return routegen_cycles + max_interconnect_cycles + bramWrLat
  }

  Predef.assert(getIDBits() <= BISMOLimits.fetchIDBits)
}

// data fetched from DRAM is combined with destination memory information from
// the address generator and made into a FetchStagePacket to be transported, one
// hop at a time, through the network into its target memory. the network uses
// the id field of the packet to determine if the target node has been reached.
class FetchStagePacket(myP: FetchStageParams) extends PrintableBundle {
  val data = Bits(width = myP.mrp.dataWidth)
  val id = UInt(width = myP.getIDBits())
  val addr = UInt(width = myP.numAddrBits)

  override def cloneType: this.type =
    new FetchStagePacket(myP).asInstanceOf[this.type]

  val printfStr = "(id = %d, addr = %d, data = %x)\n"
  val printfElems = { () ⇒ Seq(id, addr, data) }
}

class FetchInterconnect(val myP: FetchStageParams) extends Module {
  val io = new Bundle {
    val in = Decoupled(new FetchStagePacket(myP)).flip
    val node_out = Vec.fill(myP.getNumNodes()) {
      new OCMRequest(myP.mrp.dataWidth, myP.numAddrBits).asOutput
    }
  }
  // no backpressure in fetch interconnect or targets, always ready
  io.in.ready := Bool(true)
  // shift register stages along which we will carry packets
  // the valid reg chain must be initialized to all zeroes to avoid garbage writes
  val regNodeValid = Vec.fill(myP.getNumNodes()) { Reg(init = Bool(false)) }
  // the packet data reg does not have init to save LUTs
  val regNodePacket = Vec.fill(myP.getNumNodes()) { Reg(outType = io.in.bits) }

  for (i ← 0 until myP.getNumNodes()) {
    if (i == 0) {
      regNodeValid(0) := io.in.valid
      regNodePacket(0) := io.in.bits
    } else {
      regNodeValid(i) := regNodeValid(i - 1)
      regNodePacket(i) := regNodePacket(i - 1)
    }
    // addr and data are propagated to all nodes without dest. checking
    io.node_out(i).addr := regNodePacket(i).addr
    io.node_out(i).writeData := regNodePacket(i).data
    // the correct destination is the only node that gets its write enable
    io.node_out(i).writeEn := (regNodePacket(i).id === UInt(i)) & regNodeValid(i)
  }
  // uncomment to see all fetch packets passing through:
  //PrintableBundleStreamMonitor(io.in, Bool(true), "FetchInterconnect.in", true)
}

// fetch stage IO: performance counters
class FetchStagePerfIO(myP: FetchStageParams) extends Bundle {
  // clock cycles from start to done
  val cycles = UInt(OUTPUT, width = 32)

  override def cloneType: this.type =
    new FetchStagePerfIO(myP).asInstanceOf[this.type]
}

// fetch stage IO: controls to BRAM and DRAM
class FetchStageCtrlIO() extends PrintableBundle {
  // tiles per row (number of writes before going to next BRAM)
  val tiles_per_row = UInt(width = BISMOLimits.inpBufAddrBits)
  // number of blocks to fetch for each group
  val dram_block_count = UInt(width = BISMOLimits.dramBlockCountBits)
  // offset (in bytes) to start of next block in DRAM
  val dram_block_offset_bytes = UInt(width = BISMOLimits.dramBlockOffsBits)
  // size of each block (contiguous read) from DRAM
  val dram_block_size_bytes = UInt(width = BISMOLimits.dramBlockSizeBits)
  // DRAM base address for all fetch groups
  val dram_base = UInt(width = BISMOLimits.dramAddrBits)

  // base BRAM address to start from for writes
  val bram_addr_base = UInt(width = BISMOLimits.inpBufAddrBits)
  // ID range of BRAM: 0 for LHS, 1 for RHS
  val bram_id_range = Bool()
  // ID of BRAM to start from
  val bram_id_start = UInt(width = BISMOLimits.fetchIDBits)

  override def cloneType: this.type =
    new FetchStageCtrlIO().asInstanceOf[this.type]

  val printfStr = "(dram (base = %x, bsize = %d, boffs = %d, bcnt = %d), bram (idstart = %d, idrange = %d, base = %d), tiles = %d)\n"
  val printfElems = { () ⇒ Seq(dram_base, dram_block_size_bytes, dram_block_offset_bytes, dram_block_count, bram_id_start, bram_id_range, bram_addr_base, tiles_per_row) }
}

// fetch stage IO: BRAM writes
class FetchStageBRAMIO(myP: FetchStageParams) extends Bundle {
  val lhs_req = Vec.fill(myP.numLHSMems) {
    new OCMRequest(myP.mrp.dataWidth, myP.numAddrBits).asOutput
  }
  val rhs_req = Vec.fill(myP.numRHSMems) {
    new OCMRequest(myP.mrp.dataWidth, myP.numAddrBits).asOutput
  }

  /* val thr_rq = Vec.fill(myP.numLHSMems) {
    Vec.fill(myP.thsCols){
    new OCMRequest(myP.mrp.dataWidth, log2Up(myP.thrEntriesPerMem)).asOutput
  }}*/

  override def cloneType: this.type =
    new FetchStageBRAMIO(myP).asInstanceOf[this.type]
}

// fetch stage IO: DRAM reads
class FetchStageDRAMIO(myP: FetchStageParams) extends Bundle {
  val rd_req = Decoupled(new GenericMemoryRequest(myP.mrp))
  val rd_rsp = Decoupled(new GenericMemoryResponse(myP.mrp)).flip

  override def cloneType: this.type =
    new FetchStageDRAMIO(myP).asInstanceOf[this.type]
}

class FetchRouteGen(myP: FetchStageParams) extends Module {
  val io = new Bundle {
    // controls =============================================
    // prepare for new sequence (re-initialize internal state)
    val init_new = Bool(INPUT)
    // tiles per row (number of writes before going to next BRAM)
    val tiles_per_row = UInt(INPUT, width = 16)
    // base BRAM address to start from for writes
    val bram_addr_base = UInt(INPUT, width = myP.numAddrBits)
    // ID of BRAM to start from
    val bram_id_start = UInt(INPUT, width = myP.getIDBits())
    // ID range of BRAM to end at. start+range will be included
    val bram_id_range = UInt(INPUT, width = myP.getIDBits())
    // stream I/O ==========================================
    val in = Decoupled(Bits(width = myP.mrp.dataWidth)).flip
    val out = Decoupled(new FetchStagePacket(myP))
  }
  // registers for values controlling sequence generation
  val regTilesPerRow = Reg(init = UInt(0, width = 16))
  val regTilesPerRowMinusOne = Reg(init = UInt(0, width = 16))
  val regBRAMStart = Reg(init = UInt(0, width = myP.getIDBits()))
  val regBRAMRange = Reg(init = UInt(0, width = myP.getIDBits()))
  val regAddrBase = Reg(init = UInt(0, width = myP.numAddrBits))
  // internal state registers for sequence generation
  val regAddr = Reg(init = UInt(0, width = myP.numAddrBits))
  val regBRAMTarget = Reg(init = UInt(0, width = myP.getIDBits()))

  // wire up I/O streams
  io.out.valid := io.in.valid
  io.in.ready := io.out.ready
  io.out.bits.data := io.in.bits
  io.out.bits.id := regBRAMStart + regBRAMTarget
  io.out.bits.addr := regAddrBase + regAddr

  when(io.init_new) {
    // get new values for I/O signals
    regTilesPerRow := io.tiles_per_row
    regTilesPerRowMinusOne := io.tiles_per_row - UInt(1)
    regBRAMStart := io.bram_id_start
    regBRAMRange := io.bram_id_range
    regAddrBase := io.bram_addr_base
    regAddr := UInt(0)
    regBRAMTarget := UInt(0)
  }

  when(io.in.valid & io.out.ready) {
    // update route generation state
    val addr_is_at_end = (regAddr === regTilesPerRowMinusOne)
    val bram_is_at_end = (regBRAMTarget === regBRAMRange)
    val bram_incr = Mux(bram_is_at_end, UInt(0), regBRAMTarget + UInt(1))
    regAddr := Mux(addr_is_at_end, UInt(0), regAddr + UInt(1))
    regBRAMTarget := Mux(addr_is_at_end, bram_incr, regBRAMTarget)
    regAddrBase := regAddrBase + Mux(bram_is_at_end & addr_is_at_end, regTilesPerRow, UInt(0))
  }
}

class FetchDecoupledStage(val myP: FetchStageParams) extends Module {
  val io = new Bundle {
    val stage_run = Decoupled(new FetchStageCtrlIO()).flip
    val stage_done = Valid(Bool())
    val perf = new FetchStagePerfIO(myP)
    val bram = new FetchStageBRAMIO(myP)
    val dram = new FetchStageDRAMIO(myP)
  }

  /*when(io.start) {
    printf("Fetch stage runcfg: " + current_runcfg.printfStr, current_runcfg.printfElems():_*)
  }*/
  // instantiate FetchGroup components
  val reader = Module(new BlockStridedRqGen(
    mrp = myP.mrp, writeEn = false, chanID = BISMOLimits.fetchDRAMChanID
  )).io
  val routegen = Module(new FetchRouteGen(myP)).io
  val conn = Module(new FetchInterconnect(myP)).io
  val current_runcfg = Reg(init = io.stage_run.bits)

  io.stage_run.ready := Bool(false)
  io.stage_done.valid := Bool(false)
  reader.in.valid := Bool(false)
  routegen.init_new := Bool(false)

  // wire up the BlockStridedRqGen
  // outer loop
  reader.in.bits.base := current_runcfg.dram_base
  // bytes to jump between each block
  reader.in.bits.block_step := current_runcfg.dram_block_offset_bytes
  // number of blocks
  reader.in.bits.block_count := current_runcfg.dram_block_count
  // inner loop
  // bytes for beat for each block
  val bytesPerBeat = myP.mrp.dataWidth/8
  val bytesPerBurst = BISMOLimits.fetchBurstBeats * bytesPerBeat
  Predef.assert(isPow2(bytesPerBurst))
  val bytesToBurstsRightShift = log2Up(bytesPerBurst)
  reader.block_intra_step := UInt(bytesPerBurst)
  // #beats for each block
  reader.block_intra_count := current_runcfg.dram_block_size_bytes >> bytesToBurstsRightShift

  // supply read requests to DRAM from BlockStridedRqGen
  reader.out <> io.dram.rd_req
  // filter read responses from DRAM and push into route generator
  ReadRespFilter(io.dram.rd_rsp) <> routegen.in

  // wire up routing info generation
  routegen.tiles_per_row := current_runcfg.tiles_per_row
  routegen.bram_addr_base := current_runcfg.bram_addr_base
  routegen.bram_id_start := current_runcfg.bram_id_start
  val lhs_range = Reg(next=UInt(myP.numLHSMems-1, width=BISMOLimits.fetchIDBits))
  val rhs_range = Reg(next=UInt(myP.numRHSMems-1, width=BISMOLimits.fetchIDBits))
  val sel_idrange = Mux(current_runcfg.bram_id_range, rhs_range, lhs_range)
  routegen.bram_id_range := sel_idrange
  FPGAQueue(routegen.out, 2) <> conn.in

  // assign IDs to LHS and RHS memories for interconnect
  // 0...numLHSMems are the LHS IDs
  // numLHSMems..numLHSMems+numRHSMems-1 are the RHS IDs
  // numLHSMems+numRHSMems are the Thr ID
  for (i <- 0 until myP.numLHSMems) {
    val lhs_mem_ind = i
    val node_ind = i
    io.bram.lhs_req(lhs_mem_ind) := conn.node_out(node_ind)
    println(s"LHS $lhs_mem_ind assigned to node# $node_ind")
  }
  for(i <- 0 until myP.numRHSMems) {
    val rhs_mem_ind = i
    val node_ind = myP.numLHSMems + i
    io.bram.rhs_req(rhs_mem_ind) := conn.node_out(node_ind)
    println(s"RHS $rhs_mem_ind assigned to node# $node_ind")
  }

  // count responses to determine done
  val regBlockBytesReceived = Reg(init = UInt(0, 32))
  val regBlocksReceived = Reg(init = UInt(0, 32))
  val regBlockBytesAlmostFinished = Reg(init = UInt(0, 32))

  val sGetCmd :: sGenReq :: sWaitTransfer :: sWaitInterconnect :: Nil = Enum(UInt(), 4)
  val regState = Reg(init = UInt(sGetCmd))
  val regWaitInterconnect = Reg(init = UInt(0, width = 8))

  // performance counters
  // clock cycles from start to done
  val regCycles = Reg(outType = UInt(width = 32))
  when(regState === sGetCmd) {
    regCycles := UInt(0)
  } .otherwise {
    regCycles := regCycles + UInt(1)
  }
  io.perf.cycles := regCycles

  /*val prevState = Reg(next=regState)
  when(regState != prevState) {
    printf("FetchStage internal state: %d -> %d\n", prevState, regState)
  }*/

  switch(regState) {
    is(sGetCmd) {
      io.stage_run.ready := Bool(true)
      when(io.stage_run.valid) {
        current_runcfg := io.stage_run.bits
        regState := sGenReq
        regBlockBytesReceived := UInt(0)
        regBlocksReceived := UInt(0)
        regWaitInterconnect := UInt(0)
        regBlockBytesAlmostFinished := io.stage_run.bits.dram_block_size_bytes - UInt(bytesPerBeat)
      }
    }
    is(sGenReq) {
      reader.in.valid := Bool(true)
      when(reader.in.ready) {
        regState := sWaitTransfer
        // prepare routegen for a new transaction
        routegen.init_new := Bool(true)
      }
    }
    is(sWaitTransfer) {
      // wait for DMA engine to finish data transfer
      when(regBlocksReceived === current_runcfg.dram_block_count) {
        regState := sWaitInterconnect
      } .otherwise {
        when(routegen.in.fire()) {
          // count responses
          when(regBlockBytesReceived === regBlockBytesAlmostFinished) {
            regBlockBytesReceived := UInt(0)
            regBlocksReceived := regBlocksReceived + UInt(1)
          }.otherwise {
            regBlockBytesReceived := regBlockBytesReceived + UInt(bytesPerBeat)
          }
        }
      }
    }
    is(sWaitInterconnect) {
      // wait for the final data packet to travel through FetchInterconnect
      when(regWaitInterconnect === UInt(myP.getDMAtoBRAMLatency()-1)) {
        // emit done signal and back to sGetCmd
        io.stage_done.valid := Bool(true)
        regState := sGetCmd
      } .otherwise {
        regWaitInterconnect := regWaitInterconnect + UInt(1)
      }
    }
  }
}
