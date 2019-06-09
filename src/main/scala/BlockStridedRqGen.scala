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
import fpgatidbits.streams._
import fpgatidbits.ocm._

// the BlockStridedRqGen generates memory requests that span several blocks.
// each block may or may not be contiguous, and there may be strides between
// blocks as well. this is a form of 2D DMA.

class BlockStridedRqDescriptor(mrp: MemReqParams) extends Bundle {
  val base = UInt(width = mrp.addrWidth)
  val block_step = UInt(width = mrp.addrWidth)
  val block_count = UInt(width = mrp.addrWidth)

  override def cloneType: this.type =
    new BlockStridedRqDescriptor(mrp).asInstanceOf[this.type]
}

class BlockStridedRqGen(
  mrp: MemReqParams, writeEn: Boolean, chanID: Int = 0) extends Module {
  val io = new Bundle {
    val block_intra_step = UInt(INPUT, width = mrp.addrWidth)
    val block_intra_count = UInt(INPUT, width = mrp.addrWidth)
    val in = Decoupled(new BlockStridedRqDescriptor(mrp)).flip
    val out = Decoupled(new GenericMemoryRequest(mrp))
  }

  // the implementation essentially consists of two sequence generators that
  // correspond to two nested loops. the outer_sg traverses  blocks,
  // while the inner_sh traverses within the block.

  val outer_sg = Module(new MultiSeqGen(new MultiSeqGenParams(
    w = mrp.addrWidth, a = mrp.addrWidth))).io

  val inner_sg = Module(new BurstyMultiSeqGen(new BurstyMultiSeqGenParams(
    w = mrp.addrWidth, a = mrp.addrWidth, burstShift = 3))).io

  outer_sg.in.valid := io.in.valid
  /*when(io.in.valid){
    printf("[HW: 2D Req gen] Started  base %d,\n block count %d,\n block step %d\n",io.in.bits.base,io.in.bits.block_count,io.in.bits.block_step  )
  }*/
  io.in.ready := outer_sg.in.ready
  outer_sg.in.bits.init := io.in.bits.base
  outer_sg.in.bits.count := io.in.bits.block_count
  outer_sg.in.bits.step := io.in.bits.block_step

  val outer_seq = FPGAQueue(outer_sg.out, 2)
  inner_sg.in.valid := outer_seq.valid
  outer_seq.ready := inner_sg.in.ready
  inner_sg.in.bits.init := outer_seq.bits
  inner_sg.in.bits.count := io.block_intra_count
  inner_sg.in.bits.step := io.block_intra_step

  val inner_seq = FPGAQueue(inner_sg.out, 2)
  io.out.valid := inner_seq.valid
  inner_seq.ready := io.out.ready
  io.out.bits.channelID := UInt(chanID)
  io.out.bits.isWrite := Bool(writeEn)
  io.out.bits.addr := inner_seq.bits.ind
  io.out.bits.numBytes := inner_seq.bits.step
  io.out.bits.metaData := UInt(0)
  /*
  when(inner_seq.valid){
    printf("[HW: 2D Req gen] Out valid\n")
  }*/
}
