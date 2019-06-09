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

package bismo

import Chisel._
import fpgatidbits.PlatformWrapper._
import fpgatidbits.dma._
import fpgatidbits.ocm._
import fpgatidbits.streams._
import fpgatidbits.hlstools.TemplatedHLSBlackBox
import fpgatidbits.synthutils.PrintableParam

// This is the top-level source file that cobbles together the stages,
// controllers and token queues into a BISMO instance.
// The key Module here is BitSerialMatMulAccel, whose configuration is
// specified by the BitSerialMatMulParams.

// make the instantiated config options available to softare at runtime
class BitSerialMatMulHWCfg(bitsPerField: Int) extends Bundle {
  val readChanWidth = UInt(width = bitsPerField)
  val writeChanWidth = UInt(width = bitsPerField)
  val dpaDimLHS = UInt(width = bitsPerField)
  val dpaDimRHS = UInt(width = bitsPerField)
  val dpaDimCommon = UInt(width = bitsPerField)
  val lhsEntriesPerMem = UInt(width = bitsPerField)
  val rhsEntriesPerMem = UInt(width = bitsPerField)
  val accWidth = UInt(width = bitsPerField)
  val maxShiftSteps = UInt(width = bitsPerField)
  val cmdQueueEntries = UInt(width = bitsPerField)

  override def cloneType: this.type =
    new BitSerialMatMulHWCfg(bitsPerField).asInstanceOf[this.type]
}

// parameters that control the accelerator instantiation
class BitSerialMatMulParams(
  val dpaDimLHS: Int,
  val dpaDimRHS: Int,
  val dpaDimCommon: Int,
  val lhsEntriesPerMem: Int,
  val rhsEntriesPerMem: Int,
  val mrp: MemReqParams,
  val resEntriesPerMem: Int = 2,
  val bramPipelineBefore: Int = 1,
  val bramPipelineAfter: Int = 1,
  val extraRegs_DPA: Int = 0,
  val extraRegs_DPU: Int = 0,
  val extraRegs_PC: Int = 0,
  val accWidth: Int = 32,
  val maxShiftSteps: Int = 16,
  val cmdQueueEntries: Int = 16,
  val dscQueueEntries: Int = 4,
  // instruction generators
  val instrGen: Boolean = true,
  // use an optimized VHDL compressor generator
  val useVhdlCompressor: Boolean = true,
  val p2sAccelParams: StandAloneP2SParams = new StandAloneP2SParams(maxInBw = 8, nInElemPerWord = 8, outStreamSize = 64,
    fastMode = true, mrp = PYNQZ1Params.toMemReqParams() )
) extends PrintableParam {
  def estimateResources(freqMHz: Float = 200) = {
    import Math.ceil
    val a_dpu = 1.17
    val b_dpu = 44.1
    val lut_per_res = 120.1
    val lut_per_dpu = a_dpu * dpaDimCommon + b_dpu
    val lut_array = dpaDimLHS * dpaDimRHS * (lut_per_dpu + lut_per_res)
    val bram_array = ceil(dpaDimCommon / 32) * (dpaDimLHS * ceil(lhsEntriesPerMem / 1024) + dpaDimRHS * ceil(rhsEntriesPerMem / 1024))
    val binops_per_cycle = 2 * dpaDimLHS * dpaDimRHS * dpaDimCommon
    val tops_per_sec = (binops_per_cycle * freqMHz) / 1000000.0
     println("Resource predictions from cost model")
     println("=====================================")
     println(s"DPA LUT: $lut_array")
     println(s"DPA BRAM: $bram_array")
     println(s"TOPS at $freqMHz MHz: $tops_per_sec")
  }

  def headersAsList(): List[String] = {
    return List(
      "dpaLHS", "dpaRHS", "dpaCommon", "lhsMem", "rhsMem", "DRAM_rd", "DRAM_wr",
      "extraRegDPA", "extraRegDPU", "extraRegPC"
    )
  }

  def contentAsList(): List[String] = {
    return List(
      dpaDimLHS, dpaDimRHS, dpaDimCommon, lhsEntriesPerMem, rhsEntriesPerMem,
      mrp.dataWidth, mrp.dataWidth, extraRegs_DPA, extraRegs_DPU, extraRegs_PC
    ).map(_.toString)
  }

  def asHWCfgBundle(bitsPerField: Int): BitSerialMatMulHWCfg = {
    val ret = new BitSerialMatMulHWCfg(bitsPerField).asDirectionless
    ret.readChanWidth := UInt(mrp.dataWidth)
    ret.writeChanWidth := UInt(mrp.dataWidth)
    ret.dpaDimLHS := UInt(dpaDimLHS)
    ret.dpaDimRHS := UInt(dpaDimRHS)
    ret.dpaDimCommon := UInt(dpaDimCommon)
    ret.lhsEntriesPerMem := UInt(lhsEntriesPerMem)
    ret.rhsEntriesPerMem := UInt(rhsEntriesPerMem)
    ret.accWidth := UInt(accWidth)
    ret.maxShiftSteps := UInt(maxShiftSteps)
    ret.cmdQueueEntries := UInt(cmdQueueEntries)
    return ret
  }

  val fetchStageParams = new FetchStageParams(
    numLHSMems = dpaDimLHS,
    numRHSMems = dpaDimRHS,
    numAddrBits = log2Up(math.max(lhsEntriesPerMem, rhsEntriesPerMem) * dpaDimCommon / mrp.dataWidth),
    mrp = mrp, bramWrLat = bramPipelineBefore
  )
  val dpuParams = new DotProductUnitParams(
    inpWidth = dpaDimCommon, accWidth = accWidth,
    useVhdlCompressor = useVhdlCompressor
  )
  val dpaParams = new DotProductArrayParams(
    dpuParams = dpuParams, m = dpaDimLHS, n = dpaDimRHS,
    extraPipelineRegs = extraRegs_DPA
  )
  val execStageParams = new ExecStageParams(
    dpaParams = dpaParams, lhsTileMem = lhsEntriesPerMem, rhsTileMem = rhsEntriesPerMem,
    bramInRegs = bramPipelineBefore, bramOutRegs = bramPipelineAfter,
    resEntriesPerMem = resEntriesPerMem,
    tileMemAddrUnit = dpaDimCommon / mrp.dataWidth
  )
  val resultStageParams = new ResultStageParams(
    accWidth = accWidth,
    dpa_lhs = dpaDimLHS, dpa_rhs = dpaDimRHS, mrp = mrp,
    resEntriesPerMem = resEntriesPerMem,
    resMemReadLatency = 0
  )
  Predef.assert(dpaDimCommon >= mrp.dataWidth)
  Predef.assert(isPow2(dpaDimCommon / mrp.dataWidth))
  Predef.assert(log2Up(lhsEntriesPerMem) <= BISMOLimits.inpBufAddrBits)
  Predef.assert(log2Up(rhsEntriesPerMem) <= BISMOLimits.inpBufAddrBits)
  Predef.assert(log2Up(resEntriesPerMem) <= BISMOLimits.resAddrBits)
}

// Bundle to expose performance counter data to the CPU
class BitSerialMatMulPerf(myP: BitSerialMatMulParams) extends Bundle {
  val cc = UInt(OUTPUT, width = 32)
  val cc_enable = Bool(INPUT)
  val prf_fetch = new Bundle {
    val count = UInt(OUTPUT, 32)
    val sel = UInt(INPUT, log2Up(4))
  }
  val prf_exec = new Bundle {
    val count = UInt(OUTPUT, 32)
    val sel = UInt(INPUT, log2Up(4))
  }
  val prf_res = new Bundle {
    val count = UInt(OUTPUT, 32)
    val sel = UInt(INPUT, log2Up(4))
  }

  override def cloneType: this.type =
    new BitSerialMatMulPerf(myP).asInstanceOf[this.type]
}

object Stages {
  val stgFetch :: stgExec :: stgResult :: Nil = Enum(UInt(), 3)
}

class BitSerialMatMulAccel(
  val myP: BitSerialMatMulParams, p: PlatformWrapperParams
) extends GenericAccelerator(p) {
    val numMemPorts = 1
  val io = new GenericAcceleratorIF(numMemPorts, p) {
    // enable/disable execution for each stage
    val fetch_enable = Bool(INPUT)
    val exec_enable = Bool(INPUT)
    val result_enable = Bool(INPUT)
    // choose between descriptors (0) or direct instruction feed (1)
    val insOrDsc = Bool(INPUT)
    // direct instruction feed
    val ins = Decoupled(UInt(width=BISMOLimits.instrBits)).flip
    // descriptors for instruction generator
    val dsc = Decoupled(UInt(width=BISMOLimits.descrBits)).flip
    // command counts in each queue
    val fetch_op_count = UInt(OUTPUT, width = 32)
    val exec_op_count = UInt(OUTPUT, width = 32)
    val result_op_count = UInt(OUTPUT, width = 32)
    // instantiated hardware config
    val hw = new BitSerialMatMulHWCfg(32).asOutput
    // performance counter I/O
    val perf = new BitSerialMatMulPerf(myP)
    // token counts
    val tc_fe = UInt(OUTPUT, 32)
    val tc_ef = UInt(OUTPUT, 32)
    val tc_re = UInt(OUTPUT, 32)
    val tc_er = UInt(OUTPUT, 32)
    // pulse to add new tokens to the queues indicating "empty" buffers
    // are available
    val addtoken_ef = Bool(INPUT)
    val addtoken_re = Bool(INPUT)

    val enable = Bool(INPUT)
    val cmdqueue = Decoupled(new P2SCmdIO(myP.p2sAccelParams.p2sparams)).flip
    val ackqueue = Decoupled(UInt(width = 32))
  }
  io.hw := myP.asHWCfgBundle(32)
  // instantiate accelerator stages
  val fetchStage = Module(new FetchDecoupledStage(myP.fetchStageParams)).io
  // ugly hack: add child's HLS blackboxes to own list to generate dependencies
  // TODO is there some way to handle this directly in Chisel?
  val execStage_direct = Module(new ExecDecoupledStage(myP.execStageParams))
  HLSBlackBox(execStage_direct.addrgen_d)
  val execStage = execStage_direct.io
  val resultStage = Module(new ResultStage(myP.resultStageParams)).io
  // instantiate the controllers for each stage
  val fetchCtrl = Module(new FetchDecoupledController()).io
  val execCtrl = Module(new ExecDecoupledController()).io
  val resultCtrl = Module(new ResultController()).io
  // instantiate op queues
  val fetchOpQ = Module(new FPGAQueue(new BISMOFetchRunInstruction(), myP.cmdQueueEntries)).io
  val execOpQ = Module(new FPGAQueue(new BISMOExecRunInstruction(), myP.cmdQueueEntries)).io
  val resultOpQ = Module(new FPGAQueue(new BISMOResultRunInstruction(), myP.cmdQueueEntries)).io
  // instantiate tile memories
  val tilemem_lhs = Vec.fill(myP.dpaDimLHS) {
    Module(new AsymPipelinedDualPortBRAM(
      p = new OCMParameters(
        b = myP.lhsEntriesPerMem * myP.dpaDimCommon,
        rWidth = myP.dpaDimCommon, wWidth = myP.mrp.dataWidth, pts = 2, lat = 0
      ), regIn = myP.bramPipelineBefore, regOut = myP.bramPipelineAfter
    )).io
  }
  val tilemem_rhs = Vec.fill(myP.dpaDimRHS) {
    Module(new AsymPipelinedDualPortBRAM(
      p = new OCMParameters(
        b = myP.rhsEntriesPerMem * myP.dpaDimCommon,
        rWidth = myP.dpaDimCommon, wWidth = myP.mrp.dataWidth, pts = 2, lat = 0
      ), regIn = myP.bramPipelineBefore, regOut = myP.bramPipelineAfter
    )).io
  }
  // instantiate the result memory
  // TODO ResultStage actually assumes this memory can be read with zero
  // latency but current impl has latency of 1. this will cause bugs if reading
  // two different addresses in consecutive cycles.
  val resmem = Vec.fill(myP.dpaDimLHS) { Vec.fill(myP.dpaDimRHS) {
    Module(new PipelinedDualPortBRAM(
      addrBits = log2Up(myP.resEntriesPerMem), dataBits = myP.accWidth,
      regIn = 0, regOut = 0
    )).io
  }}

  // instantiate synchronization token FIFOs
  val syncFetchExec_free = Module(new FPGAQueue(Bool(), 8)).io
  val syncFetchExec_filled = Module(new FPGAQueue(Bool(), 8)).io
  val syncExecResult_free = Module(new FPGAQueue(Bool(), 8)).io
  val syncExecResult_filled = Module(new FPGAQueue(Bool(), 8)).io

  io.tc_fe := syncFetchExec_filled.count
  io.tc_ef := syncFetchExec_free.count
  io.tc_er := syncExecResult_filled.count
  io.tc_re := syncExecResult_free.count

  // helper function to wire-up DecoupledIO to DecoupledIO with pulse generator
  def enqPulseGenFromValid[TI <: Data, TO <: Bits](
    enq: DecoupledIO[TI], vld: DecoupledIO[TO]
  ) = {
    enq.valid := vld.valid & !Reg(next=vld.valid)
    enq.bits := enq.bits.fromBits(vld.bits)
    vld.ready := enq.ready
  }

  // handle direct instruction feed
  // split up single-stream feed into one for each stage
  val insDeinterleave = Module(new StreamDeinterleaverQueued(
    numDests = BISMOLimits.numStages, gen = UInt(width = BISMOLimits.instrBits),
    route = {x: UInt => new BISMOInstruction().fromBits(x).targetStage},
    capacity = 2
  )).io
  enqPulseGenFromValid(insDeinterleave.in, io.ins)

  if(myP.instrGen) {
    // instantiate the instruction generators
    val igExec = Module(HLSBlackBox(new ExecInstrGen(new ExecInstrGenParams(
      lhsEntriesPerMem = myP.lhsEntriesPerMem,
      rhsEntriesPerMem = myP.rhsEntriesPerMem,
      execToFetchLeftShift = log2Ceil(myP.dpaDimCommon / myP.mrp.dataWidth)
    )))).io
    val igFetch = Module(HLSBlackBox(new FetchInstrGen(new FetchInstrGenParams(
      dpaDimLHS = myP.dpaDimLHS, dpaDimCommon = myP.dpaDimCommon,
      dpaDimRHS = myP.dpaDimRHS,
      execToFetchLeftShift = log2Ceil(myP.dpaDimCommon / myP.mrp.dataWidth),
      lhsEntriesPerMem = myP.lhsEntriesPerMem,
      rhsEntriesPerMem = myP.rhsEntriesPerMem
    )))).io
    val igRes = Module(HLSBlackBox(new ResultInstrGen(new ResultInstrGenParams(
      dpaDimLHS = myP.dpaDimLHS, dpaDimRHS = myP.dpaDimRHS,
      accWidthBits = myP.accWidth
    )))).io
    // wire up reset differently (Vivado HLS BlackBox)
    igExec.rst_n := !this.reset
    igFetch.rst_n := !this.reset
    igRes.rst_n := !this.reset
    // instantiate descriptor queues
    val dscQ = Module(new FPGAQueue(UInt(width = BISMOLimits.descrBits), myP.dscQueueEntries)).io
    enqPulseGenFromValid(dscQ.enq, io.dsc)
    // make copies of the descriptor queue output, one for each instrgen
    StreamCopy(dscQ.deq, Seq(igFetch.in, igExec.in, igRes.in))
    val fetchInstrMix = DecoupledInputMux(io.insOrDsc, Seq(insDeinterleave.out(0), igFetch.out))
    val execInstrMix = DecoupledInputMux(io.insOrDsc, Seq(insDeinterleave.out(1), igExec.out))
    val resInstrMix = DecoupledInputMux(io.insOrDsc, Seq(insDeinterleave.out(2), igRes.out))
    // wire up instruction generator to instruction queues
    // need to use .fromBits due to difference in types (wires/content are the same)
    // fetch instrgen -> fetch op q
    fetchOpQ.enq.valid := fetchInstrMix.valid
    fetchOpQ.enq.bits := fetchOpQ.enq.bits.fromBits(fetchInstrMix.bits)
    fetchInstrMix.ready := fetchOpQ.enq.ready
    // exec instrgen -> exec op q
    execOpQ.enq.valid := execInstrMix.valid
    execOpQ.enq.bits := execOpQ.enq.bits.fromBits(execInstrMix.bits)
    execInstrMix.ready := execOpQ.enq.ready
    // result instrgen -> res op q
    resultOpQ.enq.valid := resInstrMix.valid
    resultOpQ.enq.bits := resultOpQ.enq.bits.fromBits(resInstrMix.bits)
    resInstrMix.ready := resultOpQ.enq.ready
  } else {
    // directly wire direct instruction feed into controllers
    fetchOpQ.enq.valid := insDeinterleave.out(0).valid
    fetchOpQ.enq.bits := fetchOpQ.enq.bits.fromBits(insDeinterleave.out(0).bits)
    insDeinterleave.out(0).ready := fetchOpQ.enq.ready
    // exec instrgen -> exec op q
    execOpQ.enq.valid := insDeinterleave.out(1).valid
    execOpQ.enq.bits := execOpQ.enq.bits.fromBits(insDeinterleave.out(1).bits)
    insDeinterleave.out(1).ready := execOpQ.enq.ready
    // result instrgen -> res op q
    resultOpQ.enq.valid := insDeinterleave.out(2).valid
    resultOpQ.enq.bits := resultOpQ.enq.bits.fromBits(insDeinterleave.out(2).bits)
    insDeinterleave.out(2).ready := resultOpQ.enq.ready
  }

  /*PrintableBundleStreamMonitor(fetchOpQ.enq, Bool(true), "fetchOpQ", true)
  PrintableBundleStreamMonitor(execOpQ.enq, Bool(true), "execOpQ", true)
  PrintableBundleStreamMonitor(resultOpQ.enq, Bool(true), "resultOpQ", true)*/

  /*when(fetchOpQ.enq.fire()) {
    printf("Raw fetch instr: %x \n", fetchOpQ.enq.bits.toBits())
  }
  when(execOpQ.enq.fire()) {
    printf("Raw exec instr: %x \n", execOpQ.enq.bits.toBits())
  }
  when(resultOpQ.enq.fire()) {
    printf("Raw res instr: %x \n", resultOpQ.enq.bits.toBits())
  }*/
  //printf("Op counts: %d %d %d \n", fetchOpQ.count, execOpQ.count, resultOpQ.count)

  // wire-up: command queues and pulse generators for fetch stage
  fetchCtrl.enable := io.fetch_enable
  io.fetch_op_count := fetchOpQ.count
  fetchOpQ.deq <> fetchCtrl.op

  // wire-up: command queues and pulse generators for exec stage
  execCtrl.enable := io.exec_enable
  io.exec_op_count := execOpQ.count
  execOpQ.deq <> execCtrl.op

  // wire-up: command queues and pulse generators for result stage
  resultCtrl.enable := io.result_enable
  io.result_op_count := resultOpQ.count
  resultOpQ.deq <> resultCtrl.op

  // wire-up: fetch controller and stage
  fetchCtrl.stage_run <> fetchStage.stage_run
  fetchStage.stage_done <> fetchCtrl.stage_done
  // wire-up: exec controller and stage
  execCtrl.stage_run <> execStage.stage_run
  execStage.stage_done <> execCtrl.stage_done
  // wire-up: result controller and stage
  resultStage.start := resultCtrl.start
  resultCtrl.done := resultStage.done
  resultStage.csr := resultCtrl.stageO

  // wire-up: BRAM ports (fetch and exec stages)
  // port 0 used by fetch stage for writes
  // port 1 used by execute stage for reads
  for(m <- 0 until myP.dpaDimLHS) {
    tilemem_lhs(m).ports(0).req := fetchStage.bram.lhs_req(m)
    tilemem_lhs(m).ports(1).req := execStage.tilemem.lhs_req(m)
    execStage.tilemem.lhs_rsp(m) := tilemem_lhs(m).ports(1).rsp
    //when(tilemem_lhs(m).ports(0).req.writeEn) { printf("LHS BRAM %d write: addr %d data %x\n", UInt(m), tilemem_lhs(m).ports(0).req.addr, tilemem_lhs(m).ports(0).req.writeData) }
  }
  for(m <- 0 until myP.dpaDimRHS) {
    tilemem_rhs(m).ports(0).req := fetchStage.bram.rhs_req(m)
    tilemem_rhs(m).ports(1).req := execStage.tilemem.rhs_req(m)
    execStage.tilemem.rhs_rsp(m) := tilemem_rhs(m).ports(1).rsp
    //when(tilemem_rhs(m).ports(0).req.writeEn) { printf("RHS BRAM %d write: addr %d data %x\n", UInt(m), tilemem_rhs(m).ports(0).req.addr, tilemem_rhs(m).ports(0).req.writeData) }
  }

  // wire-up: shared resource management
  // the "free" queues are initially filled by pulsing top-level I/O signals,
  // so use arbiters to mix top-level and controller-emitted signals
  // these 2 inputs are not used simultaneously in practice though
  val syncFetchExec_free_mix = Module(new Arbiter(Bool(), 2)).io
  //execCtrl.sync_out(0) <> syncFetchExec_free.enq
  execCtrl.sync_out(0) <> syncFetchExec_free_mix.in(0)
  syncFetchExec_free_mix.in(1).valid := io.addtoken_ef & !Reg(next=io.addtoken_ef)
  syncFetchExec_free_mix.in(1).bits := Bool(false) // don't care, value ignored
  syncFetchExec_free_mix.out <> syncFetchExec_free.enq

  val syncExecResult_free_mix = Module(new Arbiter(Bool(), 2)).io
  //resultCtrl.sync_out(0) <> syncExecResult_free.enq
  resultCtrl.sync_out(0) <> syncExecResult_free_mix.in(0)
  syncExecResult_free_mix.in(1).valid := io.addtoken_re & !Reg(next=io.addtoken_re)
  syncExecResult_free_mix.in(1).bits := Bool(false) // don't care, value ignored
  syncExecResult_free_mix.out <> syncExecResult_free.enq

  syncFetchExec_free.deq <> fetchCtrl.sync_in(0)
  fetchCtrl.sync_out(0) <> syncFetchExec_filled.enq
  syncFetchExec_filled.deq <> execCtrl.sync_in(0)

  syncExecResult_free.deq <> execCtrl.sync_in(1)
  execCtrl.sync_out(1) <> syncExecResult_filled.enq
  syncExecResult_filled.deq <> resultCtrl.sync_in(0)

  // wire-up: result memory (exec and result stages)
  for{
    m <- 0 until myP.dpaDimLHS
    n <- 0 until myP.dpaDimRHS
  } {
    resmem(m)(n).ports(0).req := execStage.res.req(m)(n)
    resmem(m)(n).ports(1).req := resultStage.resmem_req(m)(n)
    resultStage.resmem_rsp(m)(n) := resmem(m)(n).ports(1).rsp
  }
  // set default signature
  io.signature := makeDefaultSignature()

  // performance counters
  val regCCEnablePrev = Reg(next = io.perf.cc_enable)
  val regCC = Reg(init = UInt(0, width = 32))
  io.perf.cc := regCC
  // reset cycle counter on rising edge of cc_enable
  when(io.perf.cc_enable & !regCCEnablePrev) { regCC := UInt(0) }
  // increment cycle counter while cc_enable is high
  .elsewhen(io.perf.cc_enable & regCCEnablePrev) { regCC := regCC + UInt(1) }

  fetchCtrl.perf.start := io.perf.cc_enable
  execCtrl.perf.start := io.perf.cc_enable
  resultCtrl.perf.start := io.perf.cc_enable
  io.perf.prf_fetch <> fetchCtrl.perf
  io.perf.prf_exec <> execCtrl.perf
  io.perf.prf_res <> resultCtrl.perf

  /* TODO expose the useful ports from the monitors below:
  StreamMonitor(syncFetchExec_free.enq, io.perf.cc_enable)
  StreamMonitor(syncFetchExec_free.deq, io.perf.cc_enable)
  StreamMonitor(syncFetchExec_filled.enq, io.perf.cc_enable)
  StreamMonitor(syncFetchExec_filled.deq, io.perf.cc_enable)
  StreamMonitor(syncExecResult_free.enq, io.perf.cc_enable)
  StreamMonitor(syncExecResult_free.deq, io.perf.cc_enable)
  StreamMonitor(syncExecResult_filled.enq, io.perf.cc_enable)
  StreamMonitor(syncExecResult_filled.deq, io.perf.cc_enable)
  */

    /*** Parallel to Serial Accelerator***/
  val p2saccel = Module(new StandAloneP2SAccel(myP.p2sAccelParams, p)).io

  // instantiate and connect cmd and ack queues
  val cmdQ = Module(new FPGAQueue(io.cmdqueue.bits, 16)).io
  val ackQ = Module(new FPGAQueue(io.ackqueue.bits, 16)).io
  io.cmdqueue <> cmdQ.enq
  cmdQ.deq <> p2saccel.p2sCmd
  p2saccel.ack <> ackQ.enq
  ackQ.deq <> io.ackqueue

  // for the accelerator-facing side of the cmd and ack queues,
  // only enable transactions if io.enable is set
  p2saccel.p2sCmd.valid := cmdQ.deq.valid & io.enable
  cmdQ.deq.ready := p2saccel.p2sCmd.ready & io.enable
  p2saccel.ack.ready := ackQ.enq.ready & io.enable
  ackQ.enq.valid := p2saccel.ack.valid & io.enable

  // for the CPU-facing side of the cmd and ack queues,
  // rewire valid/ready with pulse generators to ensure single
  // enqueue/dequeue from the CPU
  cmdQ.enq.valid := io.cmdqueue.valid & !Reg(next = io.cmdqueue.valid)
  ackQ.deq.ready := io.ackqueue.ready & !Reg(next = io.ackqueue.ready)
  /*** End p2s***/

  // DRAM channels
  DecoupledInputMux(
    io.enable, Seq(fetchStage.dram.rd_req, p2saccel.memPort(0).memRdReq)
  ) <> io.memPort(0).memRdReq

  DecoupledInputMux(
    io.enable, Seq(resultStage.dram.wr_req, p2saccel.memPort(0).memWrReq)
  ) <> io.memPort(0).memWrReq

  DecoupledInputMux(
    io.enable, Seq(resultStage.dram.wr_dat, p2saccel.memPort(0).memWrDat)
  ) <> io.memPort(0).memWrDat

  io.memPort(0).memRdRsp <> DecoupledOutputDemux(
    io.enable, Seq(fetchStage.dram.rd_rsp, p2saccel.memPort(0).memRdRsp)
  )

  io.memPort(0).memWrRsp <> DecoupledOutputDemux(
    io.enable, Seq(resultStage.dram.wr_rsp, p2saccel.memPort(0).memWrRsp)
  )
}

class ResultBufParams(
  val addrBits: Int,
  val dataBits: Int,
  val regIn: Int,
  val regOut: Int
) extends PrintableParam {
  def headersAsList(): List[String] = {
    return List(
      "addBits", "dataBits", "regIn", "regOut"
    )
  }

  def contentAsList(): List[String] = {
    return List(
      addrBits, dataBits, regIn, regOut
    ).map(_.toString)
  }
}

// wrapper around PipelinedDualPortBRAM, here for characterization purposes
class ResultBuf(val myP: ResultBufParams) extends Module {
  val io = new DualPortBRAMIO(myP.addrBits, myP.dataBits)

  val mem = Module(new PipelinedDualPortBRAM(
    addrBits = myP.addrBits, dataBits = myP.dataBits,
    regIn = myP.regIn, regOut = myP.regOut
  )).io
  io <> mem
}
