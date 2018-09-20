// Copyright (c) 2018 Xilinx
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

#include <iostream>
#include <vector>
#include <cassert>
#include <time.h>
#include "platform.h"
#include "EmuTestExecInstrGenSingleMM.hpp"
#include "BISMOInstruction.hpp"
#include "StageModels.hpp"
#include "gemmbitserial/test/testhelpers.hpp"
#include "gemmbitserial/gemmbitserial.hpp"

using namespace std;

WrapperRegDriver * p;
EmuTestExecInstrGenSingleMM * t;


// generate BISMO matrix memory buffers from a bit serial matrix
// note that this is dependent on the size of the instantiated hardware
void matrix2mem(
  gemmbitserial::BitSerialMatrix & matrix, // bit serial matrix to pack
  size_t peCount,                          // number of PEs
  size_t peMemBasePtr,                     // PE memory packing start address
  size_t peMemSize,                        // number of entries in each PE mem
  StageModels::BitVector * mem             // pointer to PE memories: mem[peCount][peMemSize]
) {
  // ensure workload is aligned
  assert(matrix.nrows_a % peCount == 0);
  // ensure workload and model Dk match
  assert(sizeof(matrix.data[0]) == sizeof(StageModels::BitVector));
  // keep track of which position we are writing to for each PE memory
  vector<size_t> pe_mem_ptr(peCount, peMemBasePtr);
  // place each vector of bits into appropriate PE, interleaving rows between
  // PEs
  for(size_t b = 0; b < matrix.nbits; b++) {
    for(size_t r = 0; r < matrix.nrows_a; r++) {
      for(size_t c = 0; c < matrix.wordsPerRow(); c++) {
        size_t targetPE = r % peCount;  // interleave rows between PEs
        size_t flat_ind = targetPE * peMemSize + pe_mem_ptr[targetPE];
        mem[flat_ind] = matrix.word(b, r, c);
        // make sure we are still within bounds of the PE memory
        assert(pe_mem_ptr[targetPE] < peMemSize);
        pe_mem_ptr[targetPE]++;
      }
    }
  }
}

// create the Execute stage instruction stream for a single bit-serial MM
void ExecInstrGenSingleMM(
  // number of tiles in a single binary matrix
  // expressed in terms of the instantiated DPA size
  size_t tiles_m, size_t tiles_k, size_t tiles_n,
  // number of bits in input matrices
  size_t bits_l, size_t bits_r,
  // base addresses for buffer accesses
  size_t base_l, size_t base_r, size_t base_res,
  // number of buffers for latency hiding
  size_t nbufs_res,
  // generated instructions will be placed here
  vector<BISMOInstruction> & ret
) {
  BISMOInstruction ins;
  // all instructions are targeting the execute stage
  ins.sync.targetStage = stgExec;
  // start by acquiring input buffers
  ins.sync.isRunCfg = 0;
  ins.sync.isSendToken = 0;
  ins.sync.chanID = 0;
  ret.push_back(ins);
  // keep track of which result buffer we wrote to last
  size_t offset_res = 0;
  for(size_t m = 0; m < tiles_m; m++) {
    for(size_t n = 0; n < tiles_n; n++) {
      // starting a new result tile:
      // acquire a result buffer
      ins.sync.isRunCfg = 0;
      ins.sync.isSendToken = 0;
      ins.sync.chanID = 1;
      ret.push_back(ins);
      for(size_t l = 0; l < bits_l; l++) {
        for(size_t r = 0; r < bits_r; r++) {
          // helper variables based on current loop iteration
          bool tile_first = (l == 0) && (r == 0);
          bool tile_last = (l == bits_l-1) && (r == bits_r-1);
          size_t weight = l + r;
          bool negate = false; // TODO fix signedness
          size_t offset_l = tiles_k * (m + l * tiles_m);
          size_t offset_r = tiles_k * (n + r * tiles_n);
          // switch result buffers for latency hiding
          offset_res = (offset_res + 1) % nbufs_res;
          ins.exec.isRunCfg = 1;
          ins.exec.lhsOffset = base_l + offset_l;
          ins.exec.rhsOffset = base_r + offset_r;
          ins.exec.numTiles = tiles_k;
          ins.exec.shiftAmount = weight;
          ins.exec.negate = negate ? 1 : 0;
          // clear accumulator on first iteration of this result tile
          ins.exec.clear_before_first_accumulation = tile_first ? 1 : 0;
          // write result on first iteration of this result tile
          ins.exec.writeEn = tile_last ? 1 : 0;
          ins.exec.writeAddr = base_res + offset_res;
          ret.push_back(ins);
        }
      }
      // finished computing result tile
      // release the result buffer
      ins.sync.isRunCfg = 0;
      ins.sync.isSendToken = 1;
      ins.sync.chanID = 1;
      ret.push_back(ins);
    }
  }
  // release the input buffers
  ins.sync.isRunCfg = 0;
  ins.sync.isSendToken = 1;
  ins.sync.chanID = 0;
  ret.push_back(ins);
}

int main(int argc, char const *argv[]) {
  bool t_okay = true;
  try {
    cout << "EmuTestExecInstrGenSingleMM running" << endl;

    srand(time(NULL));

    // hardware dims for test
    const size_t Dm = 2, Dk = 4, Dn = 2;
    // define dimensions for the workload
    const size_t tiles_m = 1;
    const size_t tiles_k = 1;
    const size_t tiles_n = 1;
    const size_t bits_l = 1;
    const size_t bits_r = 1;
    const size_t base_l = 0;
    const size_t base_r = 0;
    const size_t base_res = 0;
    const size_t nbufs_res = tiles_m * tiles_n;
    const size_t nrows_lhs = Dm * tiles_m;
    const size_t nrows_rhs = Dn * tiles_n;
    const size_t ncols = Dk * tiles_k;
    const size_t mem_m = tiles_m * tiles_k * bits_l;
    const size_t mem_n = tiles_n * tiles_k * bits_r;
    const bool sgn_lhs = false;
    const bool sgn_rhs = false;

    // create a small random workload
    uint8_t * lhs = new uint8_t[nrows_lhs * ncols];
    uint8_t * rhs = new uint8_t[nrows_rhs * ncols];
    gemmbitserial::generateRandomVector(bits_l, nrows_lhs*ncols, lhs, sgn_lhs);
    gemmbitserial::generateRandomVector(bits_r, nrows_rhs*ncols, rhs, sgn_rhs);
    gemmbitserial::GEMMContext ctx = gemmbitserial::allocGEMMContext_base(
      nrows_lhs, ncols, nrows_rhs, bits_l, bits_r, sgn_lhs, sgn_rhs,
      Dm, 1, Dn, 1
    );
    ctx.lhs.importRegular(lhs);
    ctx.rhs.importRegular(rhs);
    // compute the golden result
    gemmbitserial::gemmBitSerial_generic_naive(ctx);
    gemmbitserial::printmatrix(lhs, nrows_lhs, ncols);
    cout << endl;
    gemmbitserial::printmatrix(rhs, nrows_rhs, ncols);
    cout << endl;
    gemmbitserial::printmatrix(ctx.res, nrows_lhs, nrows_rhs);

    // create instruction sequence for bit serial MM
    vector<BISMOInstruction> instrs;
    ExecInstrGenSingleMM(
      tiles_m, tiles_k, tiles_n, bits_l, bits_r, base_l,
      base_r, base_res, nbufs_res, instrs
    );

    // test generated instructions in software model
    StageModels::Accumulator * hw_acc = new StageModels::Accumulator[tiles_m*tiles_n];
    StageModels::Accumulator * hw_res = new StageModels::Accumulator[tiles_m*tiles_n*nbufs_res];
    StageModels::BitVector hw_lhs[Dm][mem_m] = {0};
    StageModels::BitVector hw_rhs[Dn][mem_n] = {0};
    const size_t hw_baseptr = 0;

    // fill memories and call the functional model with generated instructions
    matrix2mem(ctx.lhs, Dm, hw_baseptr, mem_m, (StageModels::BitVector *)hw_lhs);
    matrix2mem(ctx.rhs, Dn, hw_baseptr, mem_n, (StageModels::BitVector *)hw_rhs);
    StageModels::ExecMultiInstr<Dm, Dn, mem_m, mem_n, nbufs_res>(
      instrs, hw_lhs, hw_rhs, hw_acc, hw_res
    );

    // TODO add comparison function to compare hw_res and ctx.res

    gemmbitserial::printmatrix(hw_res, nrows_lhs, nrows_rhs);

    delete [] hw_acc;
    delete [] hw_res;

    p = initPlatform();
    t = new EmuTestExecInstrGenSingleMM(p);
    delete t;
    deinitPlatform(p);
  } catch(const char * e) {
    cout << "Exception: " << e << endl;
  }

  return t_okay ? 0 : -1;
}
