#include <stdint.h>

// defines the data layout and fields for BISMO instructions

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

ostream& operator<<(ostream& os, const BISMOSyncInstruction& dt)
{
    os << "sync: ";
    os << "stage="<< dt.targetStage << " ";
    os << "isRunCfg="<< dt.isRunCfg << " ";
    os << "isSend="<< dt.isSendToken << " ";
    os << "chanID="<< dt.chanID << std::endl;
    return os;
}

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

ostream& operator<<(ostream& os, const BISMOFetchRunInstruction& dt)
{
    os << "fetch run: ";
    os << "stage="<< dt.targetStage << " ";
    os << "isRunCfg="<< dt.isRunCfg << " ";
    os << "bram id="<< dt.bram_id_start << "+" << dt.bram_id_range << " ";
    os << "dram base=" << dt.dram_base << " bytes " << dt.dram_block_size_bytes;
    return os;
}

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

ostream& operator<<(ostream& os, const BISMOExecRunInstruction& dt)
{
    os << "exec run: ";
    os << "stage="<< dt.targetStage << " ";
    os << "isRunCfg="<< dt.isRunCfg << " ";
    os << "offset lhs rhs " << dt.lhsOffset << " " << dt.rhsOffset;
    os << "ntiles " << dt.numTiles;
    return os;
}

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

ostream& operator<<(ostream& os, const BISMOResultRunInstruction& dt)
{
    os << "result run: ";
    os << "stage="<< dt.targetStage << " ";
    os << "isRunCfg="<< dt.isRunCfg << " ";
    os << "dram_base=" << dt.dram_base;
    return os;
}

// union to store and decode all instruction types
// all instructions are currently 128 bits

union BISMOInstruction {
  uint32_t raw[4] = {0, 0, 0, 0};
  BISMOSyncInstruction sync;
  BISMOFetchRunInstruction fetch;
  BISMOExecRunInstruction exec;
  BISMOResultRunInstruction res;
};

#include <iomanip>

ostream& operator<<(ostream& os, const BISMOInstruction& dt)
{
    os.fill('0');
    os << "raw " << std::hex << setw(8) << dt.raw[0] << setw(8) << dt.raw[1] << setw(8) << dt.raw[2] << setw(8) << dt.raw[3];
    os << std::dec << std::endl;
    os.fill(' ');
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
