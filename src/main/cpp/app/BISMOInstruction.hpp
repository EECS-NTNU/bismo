#include <stdint.h>

#define BISMO_INSTR_TARGETS_BITS    2
#define BISMO_INSTR_FETCHID_BITS    5
#define BISMO_INSTR_INBFADR_BITS    16
#define BISMO_INSTR_DRAMADR_BITS    32
#define BISMO_INSTR_BLCKCNT_BITS    16
#define BISMO_INSTR_BLCKSIZ_BITS    16
#define BISMO_INSTR_MAXSHFT_BITS    5
#define BISMO_INSTR_RESADDR_BITS    1

typedef struct {
  uint64_t low  : 64;
  uint64_t high : 64;
} BISMOInstruction;

typedef struct {
  uint64_t targetStage   : BISMO_INSTR_TARGETS_BITS;
  uint64_t isRunCfg      : 1;
  uint64_t unused0      : 61;
  uint64_t unused1      : 64;
} BISMOSyncInstruction;

typedef struct {
  uint64_t targetStage   : BISMO_INSTR_TARGETS_BITS;
  uint64_t isRunCfg      : 1;
  uint64_t unused0       : 3;
  // DRAM fetch config
  // base address for all fetch groups
  uint64_t dram_base    : BISMO_INSTR_DRAMADR_BITS;
  // size of each block (contiguous read) from DRAM
  uint64_t dram_bsize   : BISMO_INSTR_BLCKSIZ_BITS;
  // offset (in bytes) to start of next block in DRAM
  uint64_t dram_boffset : BISMO_INSTR_BLCKSIZ_BITS;
  // number of blocks to fetch for each group
  uint64_t dram_bcount  : BISMO_INSTR_BLCKCNT_BITS;

  // router config
  // tiles per row (number of writes before going to next BRAM)
  uint64_t tiles_pr_row : BISMO_INSTR_INBFADR_BITS;
  // base BRAM address to start from for writes
  uint64_t bram_base    : BISMO_INSTR_INBFADR_BITS;
  // ID of BRAM to start from
  uint64_t bram_start   : BISMO_INSTR_FETCHID_BITS;
  // ID range of BRAM to end at. start+range will be included.
  uint64_t bram_range   : BISMO_INSTR_FETCHID_BITS;

} BISMOFetchRunInstruction;
