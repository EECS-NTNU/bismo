#include <stdint.h>
#include "bitfield.hpp"

union BISMOSyncInstruction {
  // storage for the instruction: 128 bits
  uint32_t raw[4] = {0, 0, 0, 0};
  // interpreted as bitfields
  BitField<0,1> isRunCfg;
  BitField<1,2> targetStage;
  BitField<3,1> isSendToken;
  BitField<4,2> chanID;
};

union BISMOFetchRunInstruction {
  // storage for the instruction: 128 bits
  uint32_t raw[4] = {0, 0, 0, 0};
  // interpreted as bitfields
  BitField<0,1> isRunCfg;
  BitField<1,2> targetStage;
  BitField<3,32> dram_base;
  BitField<35,16> dram_block_size_bytes;
  BitField<51,16> dram_block_offset_bytes;
  BitField<67,16> dram_block_count;
  BitField<83,16> tiles_per_row;
  BitField<99,16> bram_addr_base;
  BitField<115,5> bram_id_start;
  BitField<120,5> bram_id_range;
};

union BISMOExecRunInstruction {
  // storage for the instruction: 128 bits
  uint32_t raw[4] = {0, 0, 0, 0};
  // interpreted as bitfields
  BitField<0,1> isRunCfg;
  BitField<1,2> targetStage;
  BitField<3,16> lhsOffset;
  BitField<19,16> rhsOffset;
  BitField<35,16> numTiles;
  BitField<51,5> shiftAmount;
  BitField<56,1> negate;
  BitField<57,1> clear_before_first_accumulation;
  BitField<58,1> writeEn;
  BitField<59,1> writeAddr;
};

union BISMOResultRunInstruction {
  // storage for the instruction: 128 bits
  uint32_t raw[4] = {0, 0, 0, 0};
  // interpreted as bitfields
  BitField<0,1> isRunCfg;
  BitField<1,2> targetStage;
  BitField<3,32> dram_base;
  BitField<35,16> dram_skip;
  BitField<51,1> waitComplete;
  BitField<52,16> waitCompleteBytes;
  BitField<68,1> resmem_addr;
};
