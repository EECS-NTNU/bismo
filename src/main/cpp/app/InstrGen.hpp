#pragma once
#include "BISMOInstruction.hpp"
#include <vector>

namespace InstrGen {

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
  std::vector<BISMOInstruction> & ret
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

}
