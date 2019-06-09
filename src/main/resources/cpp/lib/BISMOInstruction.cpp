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

#include "BISMOInstruction.hpp"

#ifndef __SYNTHESIS__
#include <iostream>
#include <iomanip>

std::ostream& operator<<(std::ostream& os, const BISMOSyncInstruction& dt)
{
    os << "sync " << (dt.isSendToken ? "send" : "receive");
    os << " chanID="<< dt.chanID << std::endl;
    return os;
}

std::ostream& operator<<(std::ostream& os, const BISMOFetchRunInstruction& r)
{
  os << "Fetch config ============================" << std::endl;
  os << "bram_addr_base: " << r.bram_addr_base << std::endl;
  os << "bram_id_start: " << r.bram_id_start << std::endl;
  os << "bram_id_range: " << r.bram_id_range << std::endl;
  os << "tiles_per_row: " << r.tiles_per_row << std::endl;
  os << "dram_base: " << (uint64_t) r.dram_base << std::endl;
  os << "dram_block_offset_bytes: " << r.dram_block_offset_bytes << std::endl;
  os << "dram_block_size_bytes: " << r.dram_block_size_bytes << std::endl;
  os << "dram_block_count: " << r.dram_block_count << std::endl;
  os << "========================================" << std::endl;
  return os;
}

std::ostream& operator<<(std::ostream& os, const BISMOExecRunInstruction& r)
{
  os << "Exec config ============================" << std::endl;
  os << "lhsOffset: " << r.lhsOffset << std::endl;
  os << "rhsOffset: " << r.rhsOffset << std::endl;
  os << "negate: " << r.negate << std::endl;
  os << "numTiles: " << r.numTiles << std::endl;
  os << "shiftAmount: " << r.shiftAmount << std::endl;
  os << "clear_before_first_accumulation: " << r.clear_before_first_accumulation << std::endl;
  os << "writeEn: " << r.writeEn << std::endl;
  os << "writeAddr: " << r.writeAddr << std::endl;
  os << "========================================" << std::endl;
}

std::ostream& operator<<(std::ostream& os, const BISMOResultRunInstruction& r)
{
  os << "Result config ============================" << std::endl;
  os << "dram_base: " << r.dram_base << std::endl;
  os << "dram_skip: " << r.dram_skip << std::endl;
  os << "resmem_addr: " << r.resmem_addr << std::endl;
  os << "nop: " << r.nop << std::endl;
  os << "waitCompleteBytes: " << r.waitCompleteBytes << std::endl;
  os << "========================================" << std::endl;
}


std::ostream& operator<<(std::ostream& os, const BISMOInstruction& dt)
{
    /*os << dt << std::endl;
    os.fill('0');*/
    os << "raw " << dt.to_string(16) << std::endl;
    BISMOSyncInstruction sync;
    sync.fromRaw(dt);
    os << "targetStage " << sync.targetStage << " runcfg? " << sync.isRunCfg << std::endl;
    if(sync.isRunCfg == 0) {
      os << sync;
    } else {
      if(sync.targetStage == 0) {
        BISMOFetchRunInstruction fetch;
        fetch.fromRaw(dt);
        os << fetch;
      } else if(sync.targetStage == 1) {
        BISMOExecRunInstruction exec;
        exec.fromRaw(dt);
        os << exec;
      } else if(sync.targetStage == 2) {
        BISMOResultRunInstruction res;
        res.fromRaw(dt);
        os << res;
      } else {
        os << "illegal target stage";
      }
    }
    return os;
}

std::ostream& operator<<(std::ostream& os, const SingleMMDescriptor& dt)
{
  os << "[SingleMMDescriptor]" << std::endl;
  os << "===========================" << std::endl;
  os << "Raw: " << dt.asRaw().to_string(16) << std::endl;
  os << "Tiles: " << dt.tiles_m << " x " << dt.tiles_k << " x " << dt.tiles_n << std::endl;
  os << "Bits: " << (int)dt.bits_l << "b x " << (int)dt.bits_r << "b signed? " << dt.signed_l << " " << dt.signed_r << std::endl;
  os << "OCM base addresses lhs rhs res: " << dt.base_l << " " << dt.base_r << " " << dt.base_res << std::endl;
  os << "DRAM addresses lhs rhs res: " << std::hex << dt.dram_lhs << " " << dt.dram_rhs << " " << dt.dram_res << std::dec << std::endl;
  os << "#buffers for latency hiding: " << (int) dt.nbufs_fetch_exec_log2 << std::endl;
  os << "===========================" << std::endl;
  return os;
}

#endif
