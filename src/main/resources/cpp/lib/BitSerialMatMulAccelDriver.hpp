// Copyright (c) 2018 Norwegian University of Science and Technology (NTNU)
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


#ifndef BitSerialMatMulAccelDriver_H
#define BitSerialMatMulAccelDriver_H

#include <cassert>
#include <unistd.h>
#include "platform.h"
#include "BitSerialMatMulAccel.hpp"
#include <iostream>
#include "gemmbitserial/gemmbitserial.hpp"
#include "gemmbitserial/convbitserial.hpp"
#include "BISMOInstruction.hpp"

#define ASSERT_BITS(v,b)  assert(v <= (((uint64_t)1 << b) - 1));

#define CMDFIFO_CAP               16
#define FETCHEXEC_TOKENS_LOG2     2
#define FETCHEXEC_TOKENS          (1 << FETCHEXEC_TOKENS_LOG2)
#define EXECRES_TOKENS            2
#define N_CTRL_STATES             4
#define N_STAGES                  3
#define FETCH_ADDRALIGN           8
#define FETCH_SIZEALIGN           8
#define FETCH_BLOCK_MAX           (1 << (BISMO_LIMIT_DRAM_BSIZE_BITS-1))

#define max_local(x,y)                  (x > y ? x : y)
#define FETCH_ALIGN               max_local(FETCH_ADDRALIGN, FETCH_SIZEALIGN)

#define P2S_ALIGN                 64

typedef enum {
  csGetCmd = 0, csRun, csSend, csReceive
} ControllerState;

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
    delete m_accel;
  }

  // use descriptors and instruction generation hardware
  // do not use direct instruction feed
  void useDescriptors() {
    m_accel->set_insOrDsc(1);
  }

  // do not use descriptors and instruction generation hardware
  // use direct instruction feed
  void useDirectInstructionFeed() {
    m_accel->set_insOrDsc(0);
  }

  // write a descriptor into the instruction generator
  void pushSingleMMDescriptor(SingleMMDescriptor desc) {
    while(m_accel->get_dsc_ready() != 1);
    const ap_uint<BISMO_MMDESCR_BITS> raw = desc.asRaw();
    m_accel->set_dsc_bits6(raw(31, 0));
    m_accel->set_dsc_bits5(raw(63, 32));
    m_accel->set_dsc_bits4(raw(95, 64));
    m_accel->set_dsc_bits3(raw(127, 96));
    m_accel->set_dsc_bits2(raw(159, 128));
    m_accel->set_dsc_bits1(raw(191, 160));
    m_accel->set_dsc_bits0(raw(207, 192));

    m_accel->set_dsc_valid(1);
    m_accel->set_dsc_valid(0);
  }

  void pushInstruction(BISMOInstruction ins) {
    verifyInstr(ins);
    while(m_accel->get_ins_ready() != 1);
    m_accel->set_ins_bits3(ins(31, 0));
    m_accel->set_ins_bits2(ins(63, 32));
    m_accel->set_ins_bits1(ins(95, 64));
    m_accel->set_ins_bits0(ins(127, 96));
    m_accel->set_ins_valid(1);
    m_accel->set_ins_valid(0);
  }

  void verifyInstr(BISMOInstruction ins) {
    BISMOSyncInstruction sync;
    sync.fromRaw(ins);
    BISMOTargetStage stg = (BISMOTargetStage) (int) sync.targetStage;
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
    if(m_platform->platformID() == "EmuDriver" || m_platform->platformID() == "VerilatedEmuDriver") {
      // hardware emulation:
      // pretend we are running at 200 MHz for performance reporting purposes
      m_fclk = 200.0;
    } else {
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
    bool lhsSigned, bool rhsSigned, size_t extraLHSAlign = 1,
    size_t extraRHSAlign = 1, size_t extraKAlign = 1
  ) {
    const uint64_t regblock_lhs = m_cfg.dpaDimLHS * extraLHSAlign;
    const uint64_t regblock_d = extraKAlign * (m_cfg.dpaDimCommon / m_cfg.readChanWidth) * (FETCH_ALIGN / sizeof(PackedBitGroupType));
    const uint64_t regblock_rhs = m_cfg.dpaDimRHS * extraRHSAlign;
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
    return 0;
  }

  const size_t get_fetch_first_rhs_id() {
    return 0 + m_cfg.dpaDimLHS;
  }

  // do a sanity check on an instruction to fetch stage in terms of alignment and
  // out-of-bounds values
  void verifyFetchInstr(BISMOInstruction ins) {
    BISMOSyncInstruction s;
    s.fromRaw(ins);
    assert(s.targetStage == stgFetch);
    if(s.isRunCfg) {
      BISMOFetchRunInstruction f;
      f.fromRaw(ins);
      // ensure all fields are within limits
      ASSERT_BITS(f.bram_id_start, BISMO_LIMIT_FETCHID_BITS);
      ASSERT_BITS(f.bram_id_range, 1);
      ASSERT_BITS(f.bram_addr_base, BISMO_LIMIT_INBUFADDR_BITS);
      ASSERT_BITS(f.dram_base, BISMO_LIMIT_DRAMADDR_BITS);
      ASSERT_BITS(f.dram_block_size_bytes, BISMO_LIMIT_DRAM_BSIZE_BITS);
      ASSERT_BITS(f.dram_block_offset_bytes, BISMO_LIMIT_DRAM_BOFF_BITS);
      ASSERT_BITS(f.dram_block_count, BISMO_LIMIT_DRAM_BCNT_BITS);
      ASSERT_BITS(f.tiles_per_row, BISMO_LIMIT_INBUFADDR_BITS);
      // catch 0-sized transfers, may be due to overflow
      assert(f.dram_block_size_bytes != 0);

      const size_t exec_to_fetch_width_ratio = m_cfg.dpaDimCommon / m_cfg.readChanWidth;
      // ensure all DRAM accesses are aligned
      assert(((uint64_t) f.dram_base) % FETCH_ADDRALIGN == 0);
      assert(f.dram_block_offset_bytes % FETCH_ADDRALIGN == 0);
      assert(f.dram_block_size_bytes % FETCH_SIZEALIGN == 0);
      // ensure that BRAM accesses are within existing range
      assert(f.bram_id_start < get_num_fetch_nodes());
      //assert(f.bram_id_start + f.bram_id_range < get_num_fetch_nodes());
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

  // do a sanity check on an instruction for exec stage in terms of out-of-bounds values
  void verifyExecInstr(BISMOInstruction ins) {
    BISMOSyncInstruction s;
    s.fromRaw(ins);
    assert(s.targetStage == stgExec);
    if(s.isRunCfg) {
      BISMOExecRunInstruction f;
      f.fromRaw(ins);
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
    }
  }

  // do a sanity check on an instr for result stage in terms of alignment and
  // out-of-bounds values
  void verifyResultInstr(BISMOInstruction ins) {
    BISMOSyncInstruction s;
    s.fromRaw(ins);
    assert(s.targetStage == stgResult);
    if(s.isRunCfg) {
      BISMOResultRunInstruction r;
      r.fromRaw(ins);
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

  // create and return a synchronization (token queue read/write) instruction
  BISMOInstruction make_sync_instr(BISMOTargetStage stg, bool isSend, uint32_t syncChannel) {
    BISMOSyncInstruction ins;
    ins.targetStage = stg;
    ins.isRunCfg = 0;
    ins.isSendToken = isSend ? 1 : 0;
    ins.chanID = syncChannel;
    ins.unused0 = 0;
    ins.unused1 = 0;
    return ins.asRaw();
  }

  // create and return a no-operation (busy wait) instruction
  BISMOInstruction make_nop_instr(BISMOTargetStage stg) {

    if(stg != stgResult) {
      BISMOSyncInstruction ins;
      // all fields initialized to zero, including the key repetition count fields
      // just need to set target stage and isRunCfg
      ins.targetStage = stg;
      ins.isRunCfg = 1;
      return ins.asRaw();
    } else {
      BISMOResultRunInstruction ins;
      // the exception to this is the result stage, which has an explicit nop
      // field
      ins.targetStage = stg;
      ins.isRunCfg = 1;
      ins.nop = 1;
      return ins.asRaw();
    }
  }

  // add a token into the fetch-exec free queue
  void add_token_fetchexec_free() {
    m_accel->set_addtoken_ef(1);
    m_accel->set_addtoken_ef(0);
  }

  // add a token into the exec-result free queue
  void add_token_execresult_free() {
    m_accel->set_addtoken_re(1);
    m_accel->set_addtoken_re(0);
  }

  // initialize the tokens in FIFOs representing shared resources
  // this also serves as a sanity check for basic functionality in the hardware
  void init_resource_pools() {
    set_stage_enables(0, 0, 0);
    for(int i = 0; i < FETCHEXEC_TOKENS; i++) {
      add_token_fetchexec_free();
    }
    while(m_accel->get_tc_ef() != FETCHEXEC_TOKENS);

    for(int i = 0; i < EXECRES_TOKENS; i++) {
      add_token_execresult_free();
    }
    while(m_accel->get_tc_re() != EXECRES_TOKENS);
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

  void printTokenCounts() const {
    cout << "Tokens counts: ";
    cout << "ef = " << m_accel->get_tc_ef() << " ";
    cout << "fe = " << m_accel->get_tc_fe() << " ";
    cout << "er = " << m_accel->get_tc_er() << " ";
    cout << "re = " << m_accel->get_tc_re() << " ";
    cout << endl;
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


/**************************P2S driver section **************************/
void setup_p2s(
  void * accel_buf_src,
  uint32_t nbytes,
  void * accel_buf_dst,
  uint32_t rows,
  uint32_t cols,
  uint32_t bit_width,
  bool issigned
) {
  // ensure #cols is divisible by P2S_ALIGN
  assert(cols % P2S_ALIGN == 0);
  size_t col_groups = cols / P2S_ALIGN;
  m_accel->set_cmdqueue_valid(false);
  m_accel->set_cmdqueue_bits_dramBaseAddrSrc((AccelDblReg) accel_buf_src);
  // cout << "[SW] DRAM Base Addr Src" << accel_buf << endl;
  m_accel->set_cmdqueue_bits_dramBaseAddrDst((AccelDblReg) accel_buf_dst );
  m_accel->set_cmdqueue_bits_matrixRows(rows);
  m_accel->set_cmdqueue_bits_matrixColsGroup(col_groups);
  m_accel->set_cmdqueue_bits_actualPrecision(bit_width);
  m_accel->set_cmdqueue_bits_waitCompleteBytes(nbytes);
  m_accel->set_cmdqueue_bits_signed(issigned ? 1 : 0);
  // wait until cmdqueue is available
  while(m_accel->get_cmdqueue_ready() != 1);
  // pulse cmdqueue.valid
  m_accel->set_cmdqueue_valid(true);
  m_accel->set_cmdqueue_valid(false);
}

uint32_t p2s_exec_and_wait() {
  m_accel->set_enable(1);
  m_accel->set_ackqueue_ready(false);
  while(m_accel->get_ackqueue_valid() != 1);
  uint32_t ret = m_accel->get_ackqueue_bits();
  // pulse ackqueue.ready to consume ack token
  m_accel->set_ackqueue_ready(true);
  m_accel->set_ackqueue_ready(false);
  m_accel->set_enable(0);
  return ret;
}

/************************** END P2S driver section **************************/

protected:
  BitSerialMatMulAccel * m_accel;
  WrapperRegDriver * m_platform;
  HardwareCfg m_cfg;
  float m_fclk;
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
