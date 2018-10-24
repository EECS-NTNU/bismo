#pragma once

#include <stdint.h>
#include <ap_int.h>
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

  ap_uint<128> asRaw() {
    ap_uint<128> ret = 0;
    ret(1, 0) = targetStage;
    ret(2, 2) = isRunCfg;
    ret(3, 3) = isSendToken;
    ret(5, 4) = chanID;
    ret(63, 6) = unused0;
    ret(127, 64) = unused1;
    return ret;
  }
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

  ap_uint<128> asRaw() {
    ap_uint<128> ret = 0;
    ret(1, 0) = targetStage;
    ret(2, 2) = isRunCfg;
    ret(5, 3) = unused0;
    ret(10, 6) = bram_id_start;
    ret(15, 11) = bram_id_range;
    ret(31, 16) = bram_addr_base;
    ret(63, 32) = dram_base;
    ret(79, 64) = dram_block_size_bytes;
    ret(95, 80) = dram_block_offset_bytes;
    ret(111, 96) = dram_block_count;
    ret(127, 112) = tiles_per_row;

    return ret;
  }
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

  ap_uint<128> asRaw() {
    ap_uint<128> ret = 0;
    ret(1, 0) = targetStage;
    ret(2, 2) = isRunCfg;
    ret(63, 3) = unused0;
    ret(70, 64) = unused1;
    ret(86, 71) = lhsOffset;
    ret(102, 87) = rhsOffset;
    ret(118, 103) = numTiles;
    ret(123, 119) = shiftAmount;
    ret(124, 124) = negate;
    ret(125, 125) = clear_before_first_accumulation;
    ret(126, 126) = writeEn;
    ret(127, 127) = writeAddr;
    return ret;
  }
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

  ap_uint<128> asRaw() {
    ap_uint<128> ret = 0;
    ret(1, 0) = targetStage;
    ret(2, 2) = isRunCfg;
    ret(61, 3) = unused0;
    ret(62, 62) = nop;
    ret(63, 63) = resmem_addr;
    ret(95, 64) = dram_base;
    ret(111, 96) = dram_skip;
    ret(127, 112) = waitCompleteBytes;
    return ret;
  }
};

struct SingleMMDescriptor {
  // number of tiles in a single binary matrix
  // expressed in terms of the instantiated DPA size
  uint16_t tiles_m;
  uint16_t tiles_k;
  uint16_t tiles_n;
  // number of bits in input matrices
  uint8_t bits_l;
  uint8_t bits_r;
  // signedness for the input matrices
  bool signed_l;
  bool signed_r;
  // base addresses for buffer accesses
  uint16_t base_l;
  uint16_t base_r;
  uint8_t base_res;
  // number of buffers for latency hiding
  uint8_t nbufs_res;

  ap_uint<128> asRaw() {
    ap_uint<128> raw = 0;
    raw(15, 0) = tiles_m;
    raw(31, 16) = tiles_k;
    raw(47, 32) = tiles_n;
    raw(55, 48) = bits_l;
    raw(61, 56) = bits_r;
    raw(62, 62) = signed_l;
    raw(63, 63) = signed_r;
    raw(79, 64) = base_l;
    raw(95, 80) = base_r;
    raw(103, 96) = base_res;
    raw(111, 104) = nbufs_res;
    return raw;
  }

  void fromRaw(ap_uint<128> raw) {
    tiles_m = raw(15, 0);
    tiles_k = raw(31, 16);
    tiles_n = raw(47, 32);
    bits_l = raw(55, 48);
    bits_r = raw(61, 56);
    signed_l = raw(62, 62);
    signed_r = raw(63, 63);
    base_l = raw(79, 64);
    base_r = raw(95, 80);
    base_res = raw(103, 96);
    nbufs_res = raw(111, 104);
  }
};

#ifndef __SYNTHESIS__
#include <iomanip>

// union to store and decode all instruction types
// all instructions are currently 128 bits
// excluded from synthesis due to Vivado HLS problems with
// handling unions of structs
union BISMOInstruction {
  uint32_t raw[4] = {0, 0, 0, 0};
  BISMOSyncInstruction sync;
  BISMOFetchRunInstruction fetch;
  BISMOExecRunInstruction exec;
  BISMOResultRunInstruction res;
  void clear() {
    raw[0] = 0;
    raw[1] = 0;
    raw[2] = 0;
    raw[3] = 0;
  }
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
#endif
