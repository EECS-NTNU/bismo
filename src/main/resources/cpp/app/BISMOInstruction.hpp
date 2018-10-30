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
#define BISMO_MMDESCR_BITS          208
#define BISMO_INSTR_BITS            128

// NOTE: the ordering of the fields is important and should
// not be changed without making corresponding changes on the
// hardware side as well. additionally, since bit packing
// data layout is compiler implementation-dependent, it is
// necessary to run the EmuTestInstrEncoding on a new platform
// to make sure the default assumptions still hold.

struct BISMOSyncInstruction {
  ap_uint<2> targetStage;
  ap_uint<1> isRunCfg;
  ap_uint<1> isSendToken;
  ap_uint<2> chanID;
  ap_uint<58> unused0;
  ap_uint<64> unused1;

  ap_uint<BISMO_INSTR_BITS> asRaw() {
    ap_uint<BISMO_INSTR_BITS> ret = 0;
    ret(1, 0) = targetStage;
    ret(2, 2) = isRunCfg;
    ret(3, 3) = isSendToken;
    ret(5, 4) = chanID;
    ret(63, 6) = unused0;
    ret(127, 64) = unused1;
    return ret;
  }

  void fromRaw(ap_uint<BISMO_INSTR_BITS> ret) {
    targetStage = ret(1, 0);
    isRunCfg = ret(2, 2);
    isSendToken = ret(3, 3);
    chanID = ret(5, 4);
    unused0 = ret(63, 6);
    unused1 = ret(127, 64);
  }

  BISMOSyncInstruction() {
    targetStage = 0;
    isRunCfg = 0;
    isSendToken = 0;
    chanID = 0;
    unused0 = 0;
    unused1 = 0;
  }
};

struct BISMOFetchRunInstruction {
  ap_uint<2> targetStage;
  ap_uint<1> isRunCfg;
  ap_uint<3> unused0;
  ap_uint<5> bram_id_start;
  ap_uint<5> bram_id_range;
  ap_uint<16> bram_addr_base;
  ap_uint<32> dram_base;
  ap_uint<16> dram_block_size_bytes;
  ap_uint<16> dram_block_offset_bytes;
  ap_uint<16> dram_block_count;
  ap_uint<16> tiles_per_row;

  ap_uint<BISMO_INSTR_BITS> asRaw() {
    ap_uint<BISMO_INSTR_BITS> ret = 0;
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

  void fromRaw(ap_uint<BISMO_INSTR_BITS> ret) {
    targetStage = ret(1, 0);
    isRunCfg = ret(2, 2);
    unused0 = ret(5, 3);
    bram_id_start = ret(10, 6);
    bram_id_range = ret(15, 11);
    bram_addr_base = ret(31, 16);
    dram_base = ret(63, 32);
    dram_block_size_bytes = ret(79, 64);
    dram_block_offset_bytes = ret(95, 80);
    dram_block_count = ret(111, 96);
    tiles_per_row = ret(127, 112);
  }

  BISMOFetchRunInstruction() {
    targetStage = 0;
    isRunCfg = 0;
    unused0 = 0;
    bram_id_start = 0;
    bram_id_range = 0;
    bram_addr_base = 0;
    dram_base = 0;
    dram_block_size_bytes = 0;
    dram_block_offset_bytes = 0;
    dram_block_count = 0;
    tiles_per_row = 0;
  }
};

struct BISMOExecRunInstruction {
  ap_uint<2> targetStage;
  ap_uint<1> isRunCfg;
  ap_uint<61> unused0;
  ap_uint<7> unused1;
  ap_uint<16> lhsOffset;
  ap_uint<16> rhsOffset;
  ap_uint<16> numTiles;
  ap_uint<5> shiftAmount;
  ap_uint<1> negate;
  ap_uint<1> clear_before_first_accumulation;
  ap_uint<1> writeEn;
  ap_uint<1> writeAddr;

  ap_uint<BISMO_INSTR_BITS> asRaw() {
    ap_uint<BISMO_INSTR_BITS> ret = 0;
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

  void fromRaw(ap_uint<BISMO_INSTR_BITS> ret) {
    targetStage = ret(1, 0);
    isRunCfg = ret(2, 2);
    unused0 = ret(63, 3);
    unused1 = ret(70, 64);
    lhsOffset = ret(86, 71);
    rhsOffset = ret(102, 87);
    numTiles = ret(118, 103);
    shiftAmount = ret(123, 119);
    negate = ret(124, 124);
    clear_before_first_accumulation = ret(125, 125);
    writeEn = ret(126, 126);
    writeAddr = ret(127, 127);
  }

  BISMOExecRunInstruction() {
    targetStage = 0;
    isRunCfg = 0;
    unused0 = 0;
    unused1 = 0;
    lhsOffset = 0;
    rhsOffset = 0;
    numTiles = 0;
    shiftAmount = 0;
    negate = 0;
    clear_before_first_accumulation = 0;
    writeEn = 0;
    writeAddr = 0;
  }
};

struct BISMOResultRunInstruction {
  ap_uint<2> targetStage;
  ap_uint<1> isRunCfg;
  ap_uint<59> unused0;
  ap_uint<1> nop;
  ap_uint<1> resmem_addr;
  ap_uint<32> dram_base;
  ap_uint<16> dram_skip;
  ap_uint<16> waitCompleteBytes; // deprecated, do not use

  ap_uint<BISMO_INSTR_BITS> asRaw() {
    ap_uint<BISMO_INSTR_BITS> ret = 0;
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

  void fromRaw(ap_uint<BISMO_INSTR_BITS> ret) {
    targetStage = ret(1, 0);
    isRunCfg = ret(2, 2);
    unused0 = ret(61, 3);
    nop = ret(62, 62);
    resmem_addr = ret(63, 63);
    dram_base = ret(95, 64);
    dram_skip = ret(111, 96);
    waitCompleteBytes = ret(127, 112);
  }

  BISMOResultRunInstruction() {
    targetStage = 0;
    isRunCfg = 0;
    unused0 = 0;
    nop = 0;
    resmem_addr = 0;
    dram_base = 0;
    dram_skip = 0;
    waitCompleteBytes = 0; // deprecated, do not use
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
  // base pointers for source and result matrices
  uint32_t dram_lhs;
  uint32_t dram_rhs;
  uint32_t dram_res;

  ap_uint<BISMO_MMDESCR_BITS> asRaw() {
    ap_uint<BISMO_MMDESCR_BITS> raw = 0;
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
    raw(143, 112) = dram_lhs;
    raw(175, 144) = dram_rhs;
    raw(207, 176) = dram_res;
    return raw;
  }

  void fromRaw(ap_uint<BISMO_MMDESCR_BITS> raw) {
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
    dram_lhs = raw(143, 112);
    dram_rhs = raw(175, 144);
    dram_res = raw(207, 176);
  }
};

#ifndef __SYNTHESIS__
#include <iomanip>

typedef ap_uint<BISMO_INSTR_BITS> BISMOInstruction;
#define EmptyInstruction ap_uint<BISMO_INSTR_BITS>("0", 16)

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
    /*os << dt << endl;
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

#endif
