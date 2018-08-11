// Copyright (c) 2018 Xilinx
// Copyright (c) 2018 Norwegian University of Science and Technology (NTNU)
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
// * Neither the name of [project] nor the names of its
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


#ifndef BitSerialMatMulAccelDriver_H
#define BitSerialMatMulAccelDriver_H

#include <cassert>
#include <unistd.h>
#include "platform.h"
#include "BitSerialMatMulAccel.hpp"
#include <iostream>
#include "gemmbitserial/gemmbitserial.hpp"
#include "BISMOInstruction.hpp"

#define ASSERT_BITS(v,b)  assert(v <= (((uint64_t)1 << b) - 1));

#define CMDFIFO_CAP               16
#define FETCHEXEC_TOKENS          2
#define EXECRES_TOKENS            2
#define N_CTRL_STATES             4
#define N_STAGES                  3
#define FETCH_ADDRALIGN           64
#define FETCH_SIZEALIGN           8
#define FETCH_BLOCK_MAX           (1 << BISMO_LIMIT_DRAM_BSIZE_BITS)
#define MAX_INSTR_SEGS            64
#define MAX_DRAM_INSTRS           (MAX_INSTR_SEGS*m_cfg.cmdQueueEntries)
#define DRAM_INSTR_BYTES          16
#define INSTR_FETCH_GRANULARITY   (m_cfg.cmdQueueEntries/2)
//#define INSTR_FETCH_GRANULARITY   64

#define max(x,y) (x > y ? x : y)
#define FETCH_ALIGN       max(FETCH_ADDRALIGN, FETCH_SIZEALIGN)

typedef enum {
  opRun = 0, opSendToken, opReceiveToken
} OpCode;

typedef enum {
  csGetCmd = 0, csRun, csSend, csReceive
} ControllerState;

typedef struct {
  OpCode opcode;
  uint32_t syncChannel;
} Op;

typedef struct {
  uint64_t bram_addr_base;
  uint64_t bram_id_start;
  uint64_t bram_id_range;
  uint64_t dram_base;
  uint64_t dram_block_offset_bytes;
  uint64_t dram_block_size_bytes;
  uint64_t dram_block_count;
  uint64_t tiles_per_row;
} FetchRunCfg;

typedef struct {
  uint64_t lhsOffset;
  uint64_t rhsOffset;
  uint64_t negate;
  uint64_t numTiles;
  uint64_t shiftAmount;
  uint64_t clear_before_first_accumulation;
  uint64_t writeEn;
  uint64_t writeAddr;
} ExecRunCfg;

typedef struct {
  uint64_t dram_base;
  uint64_t dram_skip;
  uint64_t resmem_addr;
  uint64_t waitComplete;
  uint64_t waitCompleteBytes;
} ResultRunCfg;

typedef struct {
  uint64_t accWidth;
  uint64_t cmdQueueEntries;
  uint64_t dpaDimCommon;
  uint64_t dpaDimLHS;
  uint64_t dpaDimRHS;
  uint64_t lhsEntriesPerMem;
  uint64_t maxShiftSteps;
  uint64_t readChanWidth;
  uint64_t rhsEntriesPerMem;
  uint64_t writeChanWidth;
} HardwareCfg;

typedef uint64_t PackedBitGroupType;
typedef int32_t ResultType;

static const FetchRunCfg dummyFetchRunCfg = {0};
static const ExecRunCfg dummyExecRunCfg = {0};
static const ResultRunCfg dummyResultRunCfg = {0};

class BitSerialMatMulAccelDriver {
public:
  BitSerialMatMulAccelDriver(WrapperRegDriver * platform) {
    m_platform = platform;
    m_accel = new BitSerialMatMulAccel(m_platform);
    m_fclk = 200.0;
    update_hw_cfg();
    measure_fclk();
    // buffer allocation for instruction buffers in DRAM
    for(size_t i = 0; i < N_STAGES; i++) {
      m_host_ibuf[i] = new BISMOInstruction[MAX_DRAM_INSTRS];
      m_acc_ibuf[i] = m_platform->allocAccelBuffer(maxDRAMInstrBytes());
    }
    // set up instruction fetch threshold
    m_accel->set_if_threshold(m_cfg.cmdQueueEntries-INSTR_FETCH_GRANULARITY);

    clearInstrBuf();
  }

  ~BitSerialMatMulAccelDriver() {
    for(size_t i = 0; i < N_STAGES; i++) {
      m_platform->deallocAccelBuffer(m_acc_ibuf[i]);
      delete [] m_host_ibuf[i];
    }
  }

  // clear the DRAM instruction buffer
  void clearInstrBuf() {
    for(size_t i = 0; i < N_STAGES; i++) {
      m_icnt[i] = 0;
    }
  }

  void create_instr_stream() {
    for(size_t i = 0; i < N_STAGES; i++) {
      create_instr_stream((BISMOTargetStage) i);
    }
  }

  size_t get_completed_writes() {
    return (size_t) m_accel->get_completed_writes();
  }

  // for the given stage, create instruction fetches to pull out instrs from
  // DRAM, and ensure that instr buffer is copied to accel DRAM
  void create_instr_stream(BISMOTargetStage stg) {
    assert(stg < N_STAGES);
    size_t n_instrs = m_icnt[stg];
    void * stgBufferBase = m_acc_ibuf[stg];
    BISMOInstruction * hostInstrs = m_host_ibuf[stg];

    // round-up integer division to compute number of segments
    const size_t n_segments = (n_instrs + INSTR_FETCH_GRANULARITY - 1) / INSTR_FETCH_GRANULARITY;
    std::vector<BISMOInstruction> fetch_instrs;
    //cout << "create_instr_stream: num segments for stage " << stg << " is " << n_segments << endl;
    // for each segment of instructions, create an instruction fetch
    size_t n_instrs_left = n_instrs;
    for(size_t seg = 0; seg < n_segments; seg++) {
      size_t start_ind = INSTR_FETCH_GRANULARITY * seg;
      size_t count = n_instrs_left < INSTR_FETCH_GRANULARITY ? n_instrs_left : INSTR_FETCH_GRANULARITY;
      BISMOInstruction ins = makeInstructionFetch(stgBufferBase, start_ind, count);
      //cout << "create_instr_stream: segment " << seg << ": start = " << start_ind << " " << " count " << count << endl;
      fetch_instrs.push_back(ins);
      n_instrs_left -= count;
    }

    // copy the actual instructions to DRAM
    const size_t instr_bytes = DRAM_INSTR_BYTES * n_instrs;
    m_platform->copyBufferHostToAccel(hostInstrs, stgBufferBase, instr_bytes);

    // put the fetch instructions in OCM
    for(auto &fi : fetch_instrs) {
      switch (stg) {
        case stgFetch:
          if_fetch(fi);
          //m_accel->set_total_instr_fetch(n_instrs);
          break;
        case stgExec:
          if_exec(fi);
          //m_accel->set_total_instr_exec(n_instrs);
          break;
        case stgResult:
          if_result(fi);
          //m_accel->set_total_instr_result(n_instrs);
          break;
        default:
          cerr << "Unrecognized state in makeInstructionFetch" << endl;
          assert(0);
      }
    }
  }

  // create a fetch instruction to fetch more instructions
  BISMOInstruction makeInstructionFetch(void * base, size_t start_ind, size_t n_instrs) {
    BISMOInstruction ins;
    ins.fetch.targetStage = stgFetch;
    ins.fetch.isRunCfg = 1;
    ins.fetch.unused0 = 0;
    ins.fetch.bram_id_start = 0;
    ins.fetch.bram_id_range = 0;
    ins.fetch.bram_addr_base = 0;
    ins.fetch.dram_base = (uint64_t) base + (start_ind * DRAM_INSTR_BYTES);
    ins.fetch.dram_block_size_bytes = DRAM_INSTR_BYTES * n_instrs;
    ins.fetch.dram_block_offset_bytes = 0;
    ins.fetch.dram_block_count = 1;
    ins.fetch.tiles_per_row = 0;
    return ins;
  }

  const size_t maxDRAMInstrBytes() const {
    return MAX_DRAM_INSTRS * DRAM_INSTR_BYTES;
  }

  const size_t stageInstrCount(BISMOTargetStage stg) const {
    return m_icnt[stg];
  }

  const size_t stageInstrBytes(BISMOTargetStage stg) const {
    return m_icnt[stg] * DRAM_INSTR_BYTES;
  }

  const size_t totalInstrBytes() const {
    size_t ret = 0;
    for(size_t i = 0; i < N_STAGES; i++) {
      ret += stageInstrBytes((BISMOTargetStage) i);
    }
    return ret;
  }

  void if_fetch(BISMOInstruction ins) {
    // use regs to directly push into OCM instruction queue
    m_accel->set_if_fetch_bits0(ins.raw[3]);
    m_accel->set_if_fetch_bits1(ins.raw[2]);
    m_accel->set_if_fetch_bits2(ins.raw[1]);
    m_accel->set_if_fetch_bits3(ins.raw[0]);
    // push into fetch op FIFO when available
    while(!m_accel->get_if_fetch_ready());
    m_accel->set_if_fetch_valid(1);
    m_accel->set_if_fetch_valid(0);
  }

  void if_exec(BISMOInstruction ins) {
    // use regs to directly push into OCM instruction queue
    m_accel->set_if_exec_bits0(ins.raw[3]);
    m_accel->set_if_exec_bits1(ins.raw[2]);
    m_accel->set_if_exec_bits2(ins.raw[1]);
    m_accel->set_if_exec_bits3(ins.raw[0]);
    // push into exec op FIFO when available
    while(!m_accel->get_if_exec_ready());
    m_accel->set_if_exec_valid(1);
    m_accel->set_if_exec_valid(0);
  }

  void if_result(BISMOInstruction ins) {
    // use regs to directly push into OCM instruction queue
    m_accel->set_if_result_bits0(ins.raw[3]);
    m_accel->set_if_result_bits1(ins.raw[2]);
    m_accel->set_if_result_bits2(ins.raw[1]);
    m_accel->set_if_result_bits3(ins.raw[0]);
    // push into result op FIFO when available
    while(!m_accel->get_if_result_ready());
    m_accel->set_if_result_valid(1);
    m_accel->set_if_result_valid(0);
  }

  void push_instruction_dram(BISMOInstruction ins) {
    verifyInstr(ins);
    BISMOTargetStage stg = (BISMOTargetStage) ins.fetch.targetStage;
    assert(m_icnt[stg] < MAX_DRAM_INSTRS);

    m_host_ibuf[stg][m_icnt[stg]] = ins;
    m_icnt[stg]++;
  }

  void verifyInstr(BISMOInstruction ins) {
    BISMOTargetStage stg = (BISMOTargetStage) ins.fetch.targetStage;
    if(stg == stgFetch) {
      verifyFetchInstr(ins);
    } else if(stg == stgExec) {
      verifyExecInstr(ins);
    } else if(stg == stgResult) {
      verifyResultInstr(ins);
    } else {
      cerr << "Unrecognized target stage for instruction!" << endl;
      assert(0);
    }
  }

  void measure_fclk() {
    if(m_platform->platformID() != "EmuDriver") {
      uint32_t cc_start = perf_get_cc();
      perf_set_cc_enable(true);
      // sleep for one second of CPU time
      usleep(1000000);
      perf_set_cc_enable(false);
      uint32_t cc_end = perf_get_cc();
      // million ticks per second = fclk in MHz
      m_fclk = (float)(cc_end - cc_start) / 1000000.0;
    }
  }

  float fclk_MHz() const {
    return m_fclk;
  }

  // allocate a GEMMContext compliant with the accelerator size
  gemmbitserial::GEMMContext allocGEMMContext(
    uint64_t lhsRows, uint64_t depth, uint64_t rhsRows,
    uint64_t lhsBits, uint64_t rhsBits,
    bool lhsSigned, bool rhsSigned
  ) {
    const uint64_t regblock_lhs = m_cfg.dpaDimLHS;
    const uint64_t regblock_d = FETCH_ALIGN / sizeof(PackedBitGroupType);
    const uint64_t regblock_rhs = m_cfg.dpaDimRHS;
    const uint64_t cacheBits = 1;

    return gemmbitserial::allocGEMMContext_base(
      lhsRows, depth, rhsRows, lhsBits, rhsBits, lhsSigned, rhsSigned,
      regblock_lhs, regblock_d, regblock_rhs, cacheBits
    );
  }

  // enable/disable the cycle counter
  // cleared on rising edge (i.e. 0->1 transition)
  // increments by 1 every cycle while enabled
  void perf_set_cc_enable(bool e) {
    m_accel->set_perf_cc_enable(e ? 1 : 0);
  }

  // return cycle count
  uint32_t perf_get_cc() {
    return m_accel->get_perf_cc();
  }

  // get the number of cycles that elapsed in a given state
  // for each controller

  uint32_t perf_fetch_stats(ControllerState s) {
    m_accel->set_perf_prf_fetch_sel((uint32_t) s);
    return m_accel->get_perf_prf_fetch_count();
  }

  uint32_t perf_exec_stats(ControllerState s) {
    m_accel->set_perf_prf_exec_sel((uint32_t) s);
    return m_accel->get_perf_prf_exec_count();
  }

  uint32_t perf_result_stats(ControllerState s) {
    m_accel->set_perf_prf_res_sel((uint32_t) s);
    return m_accel->get_perf_prf_res_count();
  }

  static void printFetchRunCfg(FetchRunCfg r) {
    cout << "FetchRunCfg ============================" << endl;
    cout << "bram_addr_base: " << r.bram_addr_base << endl;
    cout << "bram_id_start: " << r.bram_id_start << endl;
    cout << "bram_id_range: " << r.bram_id_range << endl;
    cout << "tiles_per_row: " << r.tiles_per_row << endl;
    cout << "dram_base: " << (uint64_t) r.dram_base << endl;
    cout << "dram_block_offset_bytes: " << r.dram_block_offset_bytes << endl;
    cout << "dram_block_size_bytes: " << r.dram_block_size_bytes << endl;
    cout << "dram_block_count: " << r.dram_block_count << endl;
    cout << "========================================" << endl;
  }

  static void printExecRunCfg(ExecRunCfg r) {
    cout << "ExecRunCfg ============================" << endl;
    cout << "lhsOffset: " << r.lhsOffset << endl;
    cout << "rhsOffset: " << r.rhsOffset << endl;
    cout << "negate: " << r.negate << endl;
    cout << "numTiles: " << r.numTiles << endl;
    cout << "shiftAmount: " << r.shiftAmount << endl;
    cout << "clear_before_first_accumulation: " << r.clear_before_first_accumulation << endl;
    cout << "writeEn: " << r.writeEn << endl;
    cout << "writeAddr: " << r.writeAddr << endl;
    cout << "========================================" << endl;
  }

  static void printResultRunCfg(ResultRunCfg r) {
    cout << "ResultRunCfg ============================" << endl;
    cout << "dram_base: " << r.dram_base << endl;
    cout << "dram_skip: " << r.dram_skip << endl;
    cout << "resmem_addr: " << r.resmem_addr << endl;
    cout << "waitComplete: " << r.waitComplete << endl;
    cout << "waitCompleteBytes: " << r.waitCompleteBytes << endl;
    cout << "========================================" << endl;
  }

  const size_t get_lhs_total_BRAM_bytes() {
    return m_cfg.dpaDimLHS * m_cfg.lhsEntriesPerMem * m_cfg.dpaDimCommon / 8;
  }

  const size_t get_rhs_total_BRAM_bytes() {
    return m_cfg.dpaDimRHS * m_cfg.rhsEntriesPerMem * m_cfg.dpaDimCommon / 8;
  }

  const size_t get_num_fetch_nodes() {
    return (m_cfg.dpaDimLHS + m_cfg.dpaDimRHS + 1);
  }

  const size_t get_fetch_first_lhs_id() {
    return 1;
  }

  const size_t get_fetch_first_rhs_id() {
    return 1 + m_cfg.dpaDimLHS;
  }

  // do a sanity check on a FetchRunCfg in terms of alignment and
  // out-of-bounds values
  void verifyFetchInstr(BISMOInstruction ins) {
    BISMOSyncInstruction s = ins.sync;
    assert(s.targetStage == stgFetch);
    if(s.isRunCfg) {
      BISMOFetchRunInstruction f = ins.fetch;
      // ensure all fields are within limits
      ASSERT_BITS(f.bram_id_start, BISMO_LIMIT_FETCHID_BITS);
      ASSERT_BITS(f.bram_id_range, BISMO_LIMIT_FETCHID_BITS);
      ASSERT_BITS(f.bram_addr_base, BISMO_LIMIT_INBUFADDR_BITS);
      ASSERT_BITS(f.dram_base, BISMO_LIMIT_DRAMADDR_BITS);
      ASSERT_BITS(f.dram_block_size_bytes, BISMO_LIMIT_DRAM_BSIZE_BITS);
      ASSERT_BITS(f.dram_block_offset_bytes, BISMO_LIMIT_DRAM_BSIZE_BITS);
      ASSERT_BITS(f.dram_block_count, BISMO_LIMIT_DRAM_BCNT_BITS);
      ASSERT_BITS(f.tiles_per_row, BISMO_LIMIT_INBUFADDR_BITS);

      const size_t exec_to_fetch_width_ratio = m_cfg.dpaDimCommon / m_cfg.readChanWidth;
      // ensure all DRAM accesses are aligned
      assert(((uint64_t) f.dram_base) % FETCH_ADDRALIGN == 0);
      assert(f.dram_block_offset_bytes % FETCH_ADDRALIGN == 0);
      assert(f.dram_block_size_bytes % FETCH_SIZEALIGN == 0);
      // ensure that BRAM accesses are within existing range
      assert(f.bram_id_start < get_num_fetch_nodes());
      assert(f.bram_id_start + f.bram_id_range < get_num_fetch_nodes());
      if(f.bram_id_start < get_fetch_first_rhs_id()) {
        assert(f.bram_addr_base < m_cfg.lhsEntriesPerMem * exec_to_fetch_width_ratio);
      } else {
        assert(f.bram_addr_base < m_cfg.rhsEntriesPerMem * exec_to_fetch_width_ratio);
      }
    } else {
      assert(s.chanID == 0);
      assert(s.unused0 == 0);
      assert(s.unused1 == 0);
    }
  }

  // do a sanity check on a ExecRunCfg in terms of out-of-bounds values
  void verifyExecInstr(BISMOInstruction ins) {
    BISMOSyncInstruction s = ins.sync;
    assert(s.targetStage == stgExec);
    if(s.isRunCfg) {
      BISMOExecRunInstruction f = ins.exec;
      // ensure all fields are within limits
      ASSERT_BITS(f.lhsOffset, BISMO_LIMIT_INBUFADDR_BITS);
      ASSERT_BITS(f.rhsOffset, BISMO_LIMIT_INBUFADDR_BITS);
      ASSERT_BITS(f.numTiles, BISMO_LIMIT_INBUFADDR_BITS);
      ASSERT_BITS(f.shiftAmount, BISMO_LIMIT_MAXSHIFT_BITS);
      ASSERT_BITS(f.negate, 1);
      ASSERT_BITS(f.clear_before_first_accumulation, 1);
      ASSERT_BITS(f.writeEn, 1);
      ASSERT_BITS(f.writeAddr, BISMO_LIMIT_RESADDR_BITS);
    } else {
      assert(s.chanID == 0 || s.chanID == 1);
      assert(s.unused0 == 0);
      assert(s.unused1 == 0);
    }
  }

  // do a sanity check on a ResultRunCfg in terms of alignment and
  // out-of-bounds values
  void verifyResultInstr(BISMOInstruction ins) {
    BISMOSyncInstruction s = ins.sync;
    assert(s.targetStage == stgResult);
    if(s.isRunCfg) {
      BISMOResultRunInstruction r = ins.res;
      // ensure all fields are within limits
      ASSERT_BITS(r.resmem_addr, BISMO_LIMIT_RESADDR_BITS);
      ASSERT_BITS(r.dram_base, BISMO_LIMIT_DRAMADDR_BITS);
      ASSERT_BITS(r.dram_skip, BISMO_LIMIT_DRAM_BSIZE_BITS);
      // ensure all DRAM accesses are aligned to 8 bytes
      assert(((uint64_t) r.dram_base) % 8 == 0);
      assert(r.dram_skip % 8 == 0);
    } else {
      assert(s.chanID == 0);
      assert(s.unused0 == 0);
      assert(s.unused1 == 0);
    }
  }

  // get command counts in FIFOs
  const uint32_t fetch_opcount() {
    return m_accel->get_fetch_op_count();
  }
  const uint32_t exec_opcount() {
    return m_accel->get_exec_op_count();
  }
  const uint32_t res_opcount() {
    return m_accel->get_result_op_count();
  }

  // reset the accelerator
  void reset() {
    m_platform->writeReg(0, 1);
    m_platform->writeReg(0, 0);
  }

  // enable/disable the execution of each stage
  void set_stage_enables(const int fetch, const int exec, const int result) {
    m_accel->set_fetch_enable(fetch);
    m_accel->set_exec_enable(exec);
    m_accel->set_result_enable(result);
  }

  BISMOInstruction make_sync_instr(BISMOTargetStage stg, bool isSend, uint32_t syncChannel) {
    BISMOInstruction ins;
    ins.sync.targetStage = stg;
    ins.sync.isRunCfg = 0;
    ins.sync.isSendToken = isSend ? 1 : 0;
    ins.sync.chanID = syncChannel;
    ins.sync.unused0 = 0;
    ins.sync.unused1 = 0;
    return ins;
  }

  Op make_op(OpCode opcode, uint32_t syncChannel) {
    Op ret;
    ret.opcode = opcode;
    ret.syncChannel = syncChannel;
    return ret;
  }

  // push a command to the Fetch op queue
  void push_fetch_op(Op op, FetchRunCfg cfg) {
    BISMOInstruction ins;
    if(op.opcode == opRun) {
      ins.fetch.targetStage = stgFetch;
      ins.fetch.isRunCfg = 1;
      ins.fetch.unused0 = 0;
      ins.fetch.bram_id_start = cfg.bram_id_start;
      ins.fetch.bram_id_range = cfg.bram_id_range;
      ins.fetch.bram_addr_base = cfg.bram_addr_base;
      ins.fetch.dram_base = cfg.dram_base;
      ins.fetch.dram_block_size_bytes = cfg.dram_block_size_bytes;
      ins.fetch.dram_block_offset_bytes = cfg.dram_block_offset_bytes;
      ins.fetch.dram_block_count = cfg.dram_block_count;
      ins.fetch.tiles_per_row = cfg.tiles_per_row;
    } else {
      ins.sync.targetStage = stgFetch;
      ins.sync.isRunCfg = 0;
      ins.sync.isSendToken = op.opcode == opSendToken ? 1 : 0;
      ins.sync.chanID = op.syncChannel;
      ins.sync.unused0 = 0;
      ins.sync.unused1 = 0;
    }
    push_instruction_dram(ins);
  }

  // push a command to the Exec op queue
  void push_exec_op(Op op, ExecRunCfg cfg) {
    BISMOInstruction ins;
    if(op.opcode == opRun) {
      ins.exec.targetStage = stgExec;
      ins.exec.isRunCfg = 1;
      ins.exec.unused0 = 0;
      ins.exec.unused1 = 0;
      ins.exec.lhsOffset = cfg.lhsOffset;
      ins.exec.rhsOffset = cfg.rhsOffset;
      ins.exec.numTiles = cfg.numTiles;
      ins.exec.shiftAmount = cfg.shiftAmount;
      ins.exec.negate = cfg.negate;
      ins.exec.clear_before_first_accumulation = cfg.clear_before_first_accumulation;
      ins.exec.writeEn = cfg.writeEn;
      ins.exec.writeAddr = cfg.writeAddr;
    } else {
      ins.sync.targetStage = stgExec;
      ins.sync.isRunCfg = 0;
      ins.sync.isSendToken = op.opcode == opSendToken ? 1 : 0;
      ins.sync.chanID = op.syncChannel;
      ins.sync.unused0 = 0;
      ins.sync.unused1 = 0;
    }
    push_instruction_dram(ins);
  }

  // push a command to the Result op queue
  void push_result_op(Op op, ResultRunCfg cfg) {
    BISMOInstruction ins;
    if(op.opcode == opRun) {
      ins.res.targetStage = stgResult;
      ins.res.isRunCfg = 1;
      ins.res.unused0 = 0;
      ins.res.waitComplete = cfg.waitComplete;
      ins.res.resmem_addr = cfg.resmem_addr;
      ins.res.dram_base = cfg.dram_base;
      ins.res.dram_skip = cfg.dram_skip;
      ins.res.waitCompleteBytes = cfg.waitCompleteBytes;
    } else {
      ins.sync.targetStage = stgResult;
      ins.sync.isRunCfg = 0;
      ins.sync.isSendToken = op.opcode == opSendToken ? 1 : 0;
      ins.sync.chanID = op.syncChannel;
      ins.sync.unused0 = 0;
      ins.sync.unused1 = 0;
    }
    push_instruction_dram(ins);
  }

  // initialize the tokens in FIFOs representing shared resources
  // this also serves as a sanity check for basic functionality in the hardware
  void init_resource_pools() {
    // emit tokens from exec stage into the exec-fetch token queue
    for(int i = 0; i < FETCHEXEC_TOKENS; i++) {
      push_instruction_dram(make_sync_instr(stgExec, true, 0));
    }
    create_instr_stream(stgExec);
    // run instr fetches for exec + exec for the actual token init
    set_stage_enables(1, 1, 0);
    while(m_accel->get_tc_ef() != FETCHEXEC_TOKENS);
    set_stage_enables(0, 0, 0);
    // emit tokens from result stage into the result-exec token queue
    for(int i = 0; i < EXECRES_TOKENS; i++) {
      push_instruction_dram(make_sync_instr(stgResult, true, 0));
    }
    create_instr_stream(stgResult);
    // issue instr fetches for res + run res for token init
    set_stage_enables(1, 0, 1);
    while(m_accel->get_tc_re() != EXECRES_TOKENS);
    set_stage_enables(0, 0, 0);
    clearInstrBuf();
  }

  // get the instantiated hardware config
  HardwareCfg hwcfg() const {
    return m_cfg;
  }

  // update the per-stage state breakdown
  // note that the cycle counter must be enabled for this to work, e.g.
  // perf_set_cc_enable(true)
  void updateStateBreakdown() {
    // fetch the number of cycles spent in different states for each stage
    for(int i = 0; i < N_CTRL_STATES; i++) {
      m_fetch_cstate_cycles[i] = perf_fetch_stats((ControllerState) i);
    }
    for(int i = 0; i < N_CTRL_STATES; i++) {
      m_exec_cstate_cycles[i] = perf_exec_stats((ControllerState) i);
    }
    for(int i = 0; i < N_CTRL_STATES; i++) {
      m_result_cstate_cycles[i] = perf_result_stats((ControllerState) i);
    }
  }

  uint32_t getStateBreakdown(BISMOTargetStage stg, int state) {
    switch (stg) {
      case stgFetch:
        return m_fetch_cstate_cycles[state];
        break;
      case stgExec:
        return m_exec_cstate_cycles[state];
        break;
      case stgResult:
        return m_result_cstate_cycles[state];
        break;
      default:
        cerr << "getStateBreakdown: unrecognized state!" << endl;
        assert(0);
    }
  }


  // print how the stage controllers spent their time
  void printStateBreakdown() const {
    int colwidth = 11;
    std::cout << "Cycles Spent in ControllerState ========================" << std::endl;
    std::cout << std::left << std::setw(colwidth) << "Stage";
    std::cout << std::left << std::setw(colwidth) << "csGetCmd";
    std::cout << std::left << std::setw(colwidth) << "csRun";
    std::cout << std::left << std::setw(colwidth) << "csSend";
    std::cout << std::left << std::setw(colwidth) << "csReceive" << std::endl;
    // print fetch stage state cycles breakdown
    std::cout << std::left << std::setw(colwidth) << "Fetch";
    for(int i = 0; i < N_CTRL_STATES; i++) {
      std::cout << std::left << std::setw(colwidth) << m_fetch_cstate_cycles[i];
    }
    std::cout << std::endl;
    // print exec stage state cycles breakdown
    std::cout << std::left << std::setw(colwidth) << "Execute";
    for(int i = 0; i < N_CTRL_STATES; i++) {
      std::cout << std::left << std::setw(colwidth) << m_exec_cstate_cycles[i];
    }
    std::cout << std::endl;
    // print result stage state cycles breakdown
    std::cout << std::left << std::setw(colwidth) << "Result";
    for(int i = 0; i < N_CTRL_STATES; i++) {
      std::cout << std::left << std::setw(colwidth) << m_result_cstate_cycles[i];
    }
    std::cout << std::endl;
  }

  // print a summary of the hardware config
  void print_hwcfg_summary() const {
    cout << "accWidth = " << m_cfg.accWidth << endl;
    cout << "cmdQueueEntries = " << m_cfg.cmdQueueEntries << endl;
    cout << "dpaDimCommon = " << m_cfg.dpaDimCommon << endl;
    cout << "dpaDimLHS = " << m_cfg.dpaDimLHS << endl;
    cout << "dpaDimRHS = " << m_cfg.dpaDimRHS << endl;
    cout << "lhsEntriesPerMem = " << m_cfg.lhsEntriesPerMem << endl;
    cout << "maxShiftSteps = " << m_cfg.maxShiftSteps << endl;
    cout << "readChanWidth = " << m_cfg.readChanWidth << endl;
    cout << "rhsEntriesPerMem = " << m_cfg.rhsEntriesPerMem << endl;
    cout << "writeChanWidth = " << m_cfg.writeChanWidth << endl;
  }

protected:
  BitSerialMatMulAccel * m_accel;
  WrapperRegDriver * m_platform;
  HardwareCfg m_cfg;
  float m_fclk;
  // instruction buffers
  BISMOInstruction * m_host_ibuf[N_STAGES];
  void * m_acc_ibuf[N_STAGES];
  size_t m_icnt[N_STAGES];
  // performance counter variables
  uint32_t m_fetch_cstate_cycles[N_CTRL_STATES];
  uint32_t m_exec_cstate_cycles[N_CTRL_STATES];
  uint32_t m_result_cstate_cycles[N_CTRL_STATES];

  // get the instantiated hardware config from accelerator
  void update_hw_cfg() {
    m_cfg.accWidth = m_accel->get_hw_accWidth();
    m_cfg.cmdQueueEntries = m_accel->get_hw_cmdQueueEntries();
    m_cfg.dpaDimCommon = m_accel->get_hw_dpaDimCommon();
    m_cfg.dpaDimLHS = m_accel->get_hw_dpaDimLHS();
    m_cfg.dpaDimRHS = m_accel->get_hw_dpaDimRHS();
    m_cfg.lhsEntriesPerMem = m_accel->get_hw_lhsEntriesPerMem();
    m_cfg.maxShiftSteps = m_accel->get_hw_maxShiftSteps();
    m_cfg.readChanWidth = m_accel->get_hw_readChanWidth();
    m_cfg.rhsEntriesPerMem = m_accel->get_hw_rhsEntriesPerMem();
    m_cfg.writeChanWidth = m_accel->get_hw_writeChanWidth();
  }
};
#endif // BitSerialMatMulAccelDriver_H
