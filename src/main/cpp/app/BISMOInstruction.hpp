#include <stdint.h>
#include <iomanip>

// defines the data layout and fields for BISMO instructions, and routines to
// print them in human-readable representations

enum BISMOTargetStage {
  stgFetch, stgExec, stgResult
};

// these should match the values defined in BISMOLimits.scala
#define BISMO_LIMIT_FETCHID_BITS    5
#define BISMO_LIMIT_INBUFADDR_BITS  16
#define BISMO_LIMIT_DRAMADDR_BITS   32
#define BISMO_LIMIT_DRAM_BSIZE_BITS 16
#define BISMO_LIMIT_DRAM_BCNT_BITS  16
#define BISMO_LIMIT_MAXSHIFT_BITS   5
#define BISMO_LIMIT_RESADDR_BITS    1


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
  uint64_t nop : 1;
  uint64_t resmem_addr : 1;
  uint64_t dram_base : 32;
  uint64_t dram_skip : 16;
  uint64_t waitCompleteBytes : 16; // deprecated, do not use
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

ostream& operator<<(ostream& os, const BISMOSyncInstruction& dt)
{
    os << "sync " << (dt.isSendToken ? "send" : "receive");
    os << " chanID="<< dt.chanID << std::endl;
    return os;
}

ostream& operator<<(ostream& os, const BISMOFetchRunInstruction& r)
{
  os << "Fetch config ============================" << endl;
  os << "bram_addr_base: " << r.bram_addr_base << endl;
  os << "bram_id_start: " << r.bram_id_start << endl;
  os << "bram_id_range: " << r.bram_id_range << endl;
  os << "tiles_per_row: " << r.tiles_per_row << endl;
  os << "dram_base: " << (uint64_t) r.dram_base << endl;
  os << "dram_block_offset_bytes: " << r.dram_block_offset_bytes << endl;
  os << "dram_block_size_bytes: " << r.dram_block_size_bytes << endl;
  os << "dram_block_count: " << r.dram_block_count << endl;
  os << "========================================" << endl;
  return os;
}

ostream& operator<<(ostream& os, const BISMOExecRunInstruction& r)
{
  os << "Exec config ============================" << endl;
  os << "lhsOffset: " << r.lhsOffset << endl;
  os << "rhsOffset: " << r.rhsOffset << endl;
  os << "negate: " << r.negate << endl;
  os << "numTiles: " << r.numTiles << endl;
  os << "shiftAmount: " << r.shiftAmount << endl;
  os << "clear_before_first_accumulation: " << r.clear_before_first_accumulation << endl;
  os << "writeEn: " << r.writeEn << endl;
  os << "writeAddr: " << r.writeAddr << endl;
  os << "========================================" << endl;
}

ostream& operator<<(ostream& os, const BISMOResultRunInstruction& r)
{
  os << "Result config ============================" << endl;
  os << "dram_base: " << r.dram_base << endl;
  os << "dram_skip: " << r.dram_skip << endl;
  os << "resmem_addr: " << r.resmem_addr << endl;
  os << "nop: " << r.nop << endl;
  os << "waitCompleteBytes: " << r.waitCompleteBytes << endl;
  os << "========================================" << endl;
}


ostream& operator<<(ostream& os, const BISMOInstruction& dt)
{
    os.fill('0');
    os << "raw " << std::hex << setw(8) << dt.raw[0] << setw(8) << dt.raw[1] << setw(8) << dt.raw[2] << setw(8) << dt.raw[3];
    os << std::dec << std::endl;
    os.fill(' ');
    os << "targetStage " << dt.sync.targetStage << " runcfg? " << dt.sync.isRunCfg << std::endl;
    if(dt.sync.isRunCfg == 0) {
      os << dt.sync;
    } else {
      if(dt.sync.targetStage == 0) {
        os << dt.fetch;
      } else if(dt.sync.targetStage == 1) {
        os << dt.exec;
      } else if(dt.sync.targetStage == 2) {
        os << dt.res;
      } else {
        os << "illegal target stage";
      }
    }
    return os;
}
