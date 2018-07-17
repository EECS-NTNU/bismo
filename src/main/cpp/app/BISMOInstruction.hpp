#include <stdint.h>

// defines the data layout and fields for BISMO instructions

enum BISMOTargetStage {
  stgFetch, stgExec, stgResult
};

// NOTE: the ordering of the fields is important and should
// not be changed without making corresponding changes on the
// hardware side as well. additionally, since bit packing
// data layout is compiler implementation-dependent, it is
// necessary to run the EmuTestInstrEncoding on a new platform
// to make sure the default assumptions still hold.

struct BISMOSyncInstruction {
  uint64_t targetStage : 2;
  uint64_t isRunCfg : 1;
  uint64_t isSendToken : 1;
  uint64_t chanID : 2;
  uint64_t unused0 : 58;
  uint64_t unused1 : 64;
};

struct BISMOFetchRunInstruction {
  uint64_t targetStage : 2;
  uint64_t isRunCfg : 1;
  uint64_t unused0 : 3;
  uint64_t bram_id_start : 5;
  uint64_t bram_id_range : 5;
  uint64_t bram_addr_base : 16;
  uint64_t dram_base : 32;
  uint64_t dram_block_size_bytes : 16;
  uint64_t dram_block_offset_bytes : 16;
  uint64_t dram_block_count : 16;
  uint64_t tiles_per_row : 16;
};

struct BISMOExecRunInstruction {
  uint64_t targetStage : 2;
  uint64_t isRunCfg : 1;
  uint64_t unused0 : 61;
  uint64_t unused1 : 7;
  uint64_t lhsOffset : 16;
  uint64_t rhsOffset : 16;
  uint64_t numTiles : 16;
  uint64_t shiftAmount : 5;
  uint64_t negate : 1;
  uint64_t clear_before_first_accumulation : 1;
  uint64_t writeEn : 1;
  uint64_t writeAddr : 1;
};

struct BISMOResultRunInstruction {
  uint64_t targetStage : 2;
  uint64_t isRunCfg : 1;
  uint64_t unused0 : 59;
  uint64_t waitComplete : 1;
  uint64_t resmem_addr : 1;
  uint64_t dram_base : 32;
  uint64_t dram_skip : 16;
  uint64_t waitCompleteBytes : 16;
};

// union to store and decode all instruction types
// all instructions are currently 128 bits

union BISMOInstruction {
  uint32_t raw[4] = {0, 0, 0, 0};
  BISMOSyncInstruction sync;
  BISMOFetchRunInstruction fetch;
  BISMOExecRunInstruction exec;
  BISMOResultRunInstruction res;
};
