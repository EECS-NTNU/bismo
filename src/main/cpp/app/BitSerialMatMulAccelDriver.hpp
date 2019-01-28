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

#define CMDFIFO_CAP       16
#define FETCHEXEC_TOKENS  2
#define EXECRES_TOKENS    2
#define N_CTRL_STATES     4
#define FETCH_ADDRALIGN   64
#define FETCH_SIZEALIGN   8

#define max_local(x,y) (x > y ? x : y)
#define FETCH_ALIGN       max_local(FETCH_ADDRALIGN, FETCH_SIZEALIGN)

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
  uint32_t bram_addr_base;
  uint32_t bram_id_start;
  uint32_t bram_id_range;
  void * dram_base;
  uint32_t dram_block_offset_bytes;
  uint32_t dram_block_size_bytes;
  uint32_t dram_block_count;
  uint32_t tiles_per_row;
} FetchRunCfg;

typedef struct {
  uint32_t lhsOffset;
  uint32_t rhsOffset;
  uint32_t doNegate;
  uint32_t numTiles;
  uint32_t shiftAmount;
  bool doClear;
  bool writeEn;
  uint32_t writeAddr;
} ExecRunCfg;

typedef struct {
  void * dram_base;
  uint64_t dram_skip;
  uint32_t resmem_addr;
  bool waitComplete;
  uint32_t waitCompleteBytes;
} ResultRunCfg;

typedef struct {
  uint32_t accWidth;
  uint32_t cmdQueueEntries;
  uint32_t dpaDimCommon;
  uint32_t dpaDimLHS;
  uint32_t dpaDimRHS;
  uint32_t lhsEntriesPerMem;
  uint32_t maxShiftSteps;
  uint32_t readChanWidth;
  uint32_t rhsEntriesPerMem;
  uint32_t writeChanWidth;
} HardwareCfg;

typedef uint64_t PackedBitGroupType;
typedef int32_t ResultType;

class BitSerialMatMulAccelDriver {
public:
  BitSerialMatMulAccelDriver(WrapperRegDriver * platform) {
    m_platform = platform;
    m_accel = new BitSerialMatMulAccel(m_platform);
    m_fclk = 200.0;
    update_hw_cfg();
    measure_fclk();
  }
  ~BitSerialMatMulAccelDriver() {
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
    cout << "doNegate: " << r.doNegate << endl;
    cout << "numTiles: " << r.numTiles << endl;
    cout << "shiftAmount: " << r.shiftAmount << endl;
    cout << "doClear: " << r.doClear << endl;
    cout << "writeEn: " << r.writeEn << endl;
    cout << "writeAddr: " << r.writeAddr << endl;
    cout << "========================================" << endl;
  }

  const size_t get_lhs_total_BRAM_bytes() {
    return m_cfg.dpaDimLHS * m_cfg.lhsEntriesPerMem * m_cfg.dpaDimCommon / 8;
  }

  const size_t get_rhs_total_BRAM_bytes() {
    return m_cfg.dpaDimRHS * m_cfg.rhsEntriesPerMem * m_cfg.dpaDimCommon / 8;
  }

  const size_t get_fetch_nodes_per_group() {
    return (m_cfg.dpaDimLHS + m_cfg.dpaDimRHS);
  }

  const size_t get_fetch_first_lhs_id() {
    return 0;
  }

  const size_t get_fetch_first_rhs_id() {
    return m_cfg.dpaDimLHS;
  }

  // do a sanity check on a FetchRunCfg in terms of alignment and
  // out-of-bounds values
  void verifyFetchRunCfg(FetchRunCfg f) {
    const size_t exec_to_fetch_width_ratio = m_cfg.dpaDimCommon / m_cfg.readChanWidth;
    // ensure all DRAM accesses are aligned
    assert(((uint64_t) f.dram_base) % FETCH_ADDRALIGN == 0);
    assert(f.dram_block_offset_bytes % FETCH_ADDRALIGN == 0);
    assert(f.dram_block_size_bytes % FETCH_SIZEALIGN == 0);
    // ensure that BRAM accesses are within existing range
    assert(f.bram_id_start < get_fetch_nodes_per_group());
    assert(f.bram_id_start + f.bram_id_range < get_fetch_nodes_per_group());
    if(f.bram_id_start < get_fetch_first_rhs_id()) {
      assert(f.bram_addr_base < m_cfg.lhsEntriesPerMem * exec_to_fetch_width_ratio);
    } else {
      assert(f.bram_addr_base < m_cfg.rhsEntriesPerMem * exec_to_fetch_width_ratio);
    }
  }

  // do a sanity check on a ResultRunCfg in terms of alignment and
  // out-of-bounds values
  void verifyResultRunCfg(ResultRunCfg r) {
    // ensure all DRAM accesses are aligned to 8 bytes
    assert(((uint64_t) r.dram_base) % 8 == 0);
    assert(r.dram_skip % 8 == 0);
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

  // check whether it's possible to write a new element into a queue
  const bool fetch_op_full() {
    return m_accel->get_fetch_op_ready() == 1 ? false : true;
  }
  const bool exec_op_full() {
    return m_accel->get_exec_op_ready() == 1 ? false : true;
  }
  const bool result_op_full() {
    return m_accel->get_result_op_ready() == 1 ? false : true;
  }
  const bool fetch_runcfg_full() {
    return m_accel->get_fetch_runcfg_ready() == 1 ? false : true;
  }
  const bool exec_runcfg_full() {
    return m_accel->get_exec_runcfg_ready() == 1 ? false : true;
  }
  const bool result_runcfg_full() {
    return m_accel->get_result_runcfg_ready() == 1 ? false : true;
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

  Op make_op(OpCode opcode, uint32_t syncChannel) {
    Op ret;
    ret.opcode = opcode;
    ret.syncChannel = syncChannel;
    return ret;
  }

  // push a command to the Fetch op queue
  void push_fetch_op(Op op) {
    m_accel->set_fetch_op_bits_opcode((AccelReg) op.opcode);
    m_accel->set_fetch_op_bits_token_channel(op.syncChannel);
    // push into fetch op FIFO
    while(fetch_op_full()){cout<<"Fetch OP Full"<<endl;};
    //assert(!fetch_op_full());
    m_accel->set_fetch_op_valid(1);
    m_accel->set_fetch_op_valid(0);
  }

  // push a command to the Exec op queue
  void push_exec_op(Op op) {
    m_accel->set_exec_op_bits_opcode((AccelReg) op.opcode);
    m_accel->set_exec_op_bits_token_channel(op.syncChannel);
    // push into exec op FIFO
    while(exec_op_full()){cout<<"Exec OP Full"<<endl;};
    // assert(!exec_op_full());
    m_accel->set_exec_op_valid(1);
    m_accel->set_exec_op_valid(0);
  }

  // push a command to the Result op queue
  void push_result_op(Op op) {
    m_accel->set_result_op_bits_opcode((AccelReg) op.opcode);
    m_accel->set_result_op_bits_token_channel(op.syncChannel);
    // push into result op FIFO
    while(result_op_full()){cout<<"Result OP Full"<<endl;};
    // assert(!result_op_full());
    m_accel->set_result_op_valid(1);
    m_accel->set_result_op_valid(0);
  }

  // push a command to the Fetch runcfg queue
  void push_fetch_runcfg(FetchRunCfg cfg) {
    // set up all the fields for the runcfg
    m_accel->set_fetch_runcfg_bits_bram_addr_base(cfg.bram_addr_base);
    m_accel->set_fetch_runcfg_bits_bram_id_range(cfg.bram_id_range);
    m_accel->set_fetch_runcfg_bits_bram_id_start(cfg.bram_id_start);
    m_accel->set_fetch_runcfg_bits_dram_base((AccelDblReg) cfg.dram_base);
    m_accel->set_fetch_runcfg_bits_dram_block_offset_bytes(cfg.dram_block_offset_bytes);
    m_accel->set_fetch_runcfg_bits_dram_block_size_bytes(cfg.dram_block_size_bytes);
    m_accel->set_fetch_runcfg_bits_dram_block_count(cfg.dram_block_count);
    // hw limitation: tiles_per_row is internally 16 bits
    assert(cfg.tiles_per_row < (1 << 16));
    m_accel->set_fetch_runcfg_bits_tiles_per_row(cfg.tiles_per_row);
    // push to runcfg FIFO
    while(fetch_runcfg_full()){cout<<"Fetch runcfg Full"<<endl;};

    // assert(!fetch_runcfg_full());
    m_accel->set_fetch_runcfg_valid(1);
    m_accel->set_fetch_runcfg_valid(0);
  }

  // push a command to the Exec runcfg queue
  void push_exec_runcfg(ExecRunCfg cfg) {
    // set up all the fields for the runcfg
    m_accel->set_exec_runcfg_bits_clear_before_first_accumulation(cfg.doClear ? 1 : 0);
    m_accel->set_exec_runcfg_bits_lhsOffset(cfg.lhsOffset);
    m_accel->set_exec_runcfg_bits_negate(cfg.doNegate);
    m_accel->set_exec_runcfg_bits_numTiles(cfg.numTiles);
    m_accel->set_exec_runcfg_bits_rhsOffset(cfg.rhsOffset);
    m_accel->set_exec_runcfg_bits_shiftAmount(cfg.shiftAmount);
    m_accel->set_exec_runcfg_bits_writeEn(cfg.writeEn);
    m_accel->set_exec_runcfg_bits_writeAddr(cfg.writeAddr);
    while(exec_runcfg_full()){cout<<"Exec runcfg Full"<<endl;};
    // assert(!exec_runcfg_full());
    // push to runcfg FIFO
    m_accel->set_exec_runcfg_valid(1);
    m_accel->set_exec_runcfg_valid(0);
  }

  // push a command to the Result runcfg queue
  void push_result_runcfg(ResultRunCfg cfg) {
    // set up all the fields for the command
    m_accel->set_result_runcfg_bits_dram_base((AccelDblReg) cfg.dram_base);
    m_accel->set_result_runcfg_bits_dram_skip(cfg.dram_skip);
    m_accel->set_result_runcfg_bits_resmem_addr(cfg.resmem_addr);
    m_accel->set_result_runcfg_bits_waitComplete(cfg.waitComplete ? 1 : 0);
    m_accel->set_result_runcfg_bits_waitCompleteBytes(cfg.waitCompleteBytes);
    // push to runcfg FIFO
    while(result_runcfg_full()){cout<<"Result runcfg Full"<<endl;};
    // assert(!result_runcfg_full());
    m_accel->set_result_runcfg_valid(1);
    m_accel->set_result_runcfg_valid(0);
  }

  // initialize the tokens in FIFOs representing shared resources
  void init_resource_pools() {
    set_stage_enables(0, 0, 0);
    for(int i = 0; i < FETCHEXEC_TOKENS; i++) {
      push_exec_op(make_op(opSendToken, 0));
    }
    assert(m_accel->get_exec_op_count() == FETCHEXEC_TOKENS);
    set_stage_enables(0, 1, 0);
    while(m_accel->get_exec_op_count() != 0);

    set_stage_enables(0, 0, 0);
    for(int i = 0; i < EXECRES_TOKENS; i++) {
      push_result_op(make_op(opSendToken, 0));
    }
    assert(m_accel->get_result_op_count() == EXECRES_TOKENS);
    set_stage_enables(0, 0, 1);
    while(m_accel->get_result_op_count() != 0);
    set_stage_enables(0, 0, 0);
  }

  // get the instantiated hardware config
  HardwareCfg hwcfg() const {
    return m_cfg;
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
