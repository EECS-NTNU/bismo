#pragma once
#include <stdint.h>
#include "BISMOInstruction.hpp"
#include <vector>
#include <cassert>

namespace InstrGen {

template <
  // matmul array dimensions: rows, common, cols
  size_t M, size_t K, size_t N,
  // exec-to-fetch left-shift ratio: log2(K / fetch width)
  size_t ETF_S
>
void FetchInstrGenSingleMM(
  // desciptor for the MM operation
  SingleMMDescriptor in,
  // generated instructions will be placed here
  std::vector<BISMOInstruction> & out
) {
  // TODO handle off-chip matrices
  // TODO fetch-exec concurrency
  // TODO no-fetch mode? (if dram addr = -1 just generate sync)
  BISMOSyncInstruction sync;
  BISMOFetchRunInstruction fetch;
  const size_t lhs_tile_bytes = (M*K)/8;
  const size_t rhs_tile_bytes = (N*K)/8;
  // acquire buffers
  sync.targetStage = stgFetch;
  sync.isRunCfg = 0;
  sync.isSendToken = 0;
  sync.chanID = 0;
  out.push_back(sync.asRaw());

  // fetch LHS matrix
  fetch.targetStage = stgFetch;
  fetch.isRunCfg = 1;
  fetch.bram_id_start = 0;
  fetch.bram_id_range = M-1;
  fetch.bram_addr_base = in.base_l << ETF_S;
  fetch.tiles_per_row = in.tiles_k << ETF_S;
  fetch.dram_base = in.dram_lhs;
  // TODO fix block calculation here
  fetch.dram_block_size_bytes = in.tiles_m * in.tiles_k * in.bits_l * lhs_tile_bytes;
  fetch.dram_block_offset_bytes = 0;
  fetch.dram_block_count = 1;
  out.push_back(fetch.asRaw());

  // fetch RHS matrix
  fetch.targetStage = stgFetch;
  fetch.isRunCfg = 1;
  fetch.bram_id_start = M;
  fetch.bram_id_range = N-1;
  fetch.bram_addr_base = in.base_r << ETF_S;
  fetch.tiles_per_row = in.tiles_k << ETF_S;
  fetch.dram_base = in.dram_rhs;
  // TODO fix block calculation here
  fetch.dram_block_size_bytes = in.tiles_n * in.tiles_k * in.bits_r * rhs_tile_bytes;
  fetch.dram_block_offset_bytes = 0;
  fetch.dram_block_count = 1;
  out.push_back(fetch.asRaw());

  // release buffers
  sync.targetStage = stgFetch;
  sync.isRunCfg = 0;
  sync.isSendToken = 1;
  sync.chanID = 0;
  out.push_back(sync.asRaw());
}


// create the Execute stage instruction stream for a single bit-serial MM
void ExecInstrGenSingleMM(
  // desciptor for the MM operation
  SingleMMDescriptor in,
  // generated instructions will be placed here
  std::vector<BISMOInstruction> & out
) {
  // single-bit signed is used to indicate bipolar (-1, +1) which is
  // currently not supported:
  assert(!(in.bits_l == 1 && in.signed_l));
  assert(!(in.bits_r == 1 && in.signed_r));
  BISMOSyncInstruction sync;
  BISMOExecRunInstruction exec;
  sync.targetStage = stgExec;
  // start by acquiring input buffers
  sync.isRunCfg = 0;
  sync.isSendToken = 0;
  sync.chanID = 0;
  out.push_back(sync.asRaw());
  // keep track of which result buffer we wrote to last
  size_t offset_res = 0;
  for(size_t m = 0; m < in.tiles_m; m++) {
    for(size_t n = 0; n < in.tiles_n; n++) {
      // starting a new result tile:
      // acquire a result buffer
      sync.targetStage = stgExec;
      sync.isRunCfg = 0;
      sync.isSendToken = 0;
      sync.chanID = 1;
      out.push_back(sync.asRaw());
      for(size_t l = 0; l < in.bits_l; l++) {
        for(size_t r = 0; r < in.bits_r; r++) {
          // helper variables based on current loop iteration
          bool tile_first = (l == 0) && (r == 0);
          bool tile_last = (l == in.bits_l-1) && (r == in.bits_r-1);
          size_t weight = l + r;
          // whether the current bit position is negative for
          // the input matrices
          bool neg_l = (l == in.bits_l-1) && in.signed_l;
          bool neg_r = (r == in.bits_r-1) && in.signed_r;
          bool negate = neg_l ^ neg_r;
          size_t offset_l = in.tiles_k * (m + l * in.tiles_m);
          size_t offset_r = in.tiles_k * (n + r * in.tiles_n);
          // switch result buffers for latency hiding
          offset_res = (offset_res + 1) % in.nbufs_res;
          exec.targetStage = stgExec;
          exec.isRunCfg = 1;
          exec.lhsOffset = in.base_l + offset_l;
          exec.rhsOffset = in.base_r + offset_r;
          exec.numTiles = in.tiles_k;
          exec.shiftAmount = weight;
          exec.negate = negate ? 1 : 0;
          // clear accumulator on first iteration of this result tile
          exec.clear_before_first_accumulation = tile_first ? 1 : 0;
          // write result on first iteration of this result tile
          exec.writeEn = tile_last ? 1 : 0;
          exec.writeAddr = in.base_res + offset_res;
          out.push_back(exec.asRaw());
        }
      }
      // finished computing result tile
      // release the result buffer
      sync.targetStage = stgExec;
      sync.isRunCfg = 0;
      sync.isSendToken = 1;
      sync.chanID = 1;
      out.push_back(sync.asRaw());
    }
  }
  // release the input buffers
  sync.isRunCfg = 0;
  sync.isSendToken = 1;
  sync.chanID = 0;
  out.push_back(sync.asRaw());
}

}
