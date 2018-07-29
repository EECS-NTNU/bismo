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

package bismo

import Chisel._
import fpgatidbits.PlatformWrapper._
import fpgatidbits.dma._
import fpgatidbits.ocm._
import fpgatidbits.streams._

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
  // do not instantiate the shift stage
  val noShifter: Boolean = false,
  // do not instantiate the negate stage
  val noNegate: Boolean = false
) extends PrintableParam {
  def estimateResources() = {
    import Math.ceil
    val a_dpu = 2.04
    val b_dpu = 109.41
    val lut_per_res = 120.1
    val lut_per_dpu = a_dpu * dpaDimCommon + b_dpu
    val lut_array = dpaDimLHS * dpaDimRHS * (lut_per_dpu + lut_per_res)
    val bram_array = ceil(dpaDimCommon / 32) * (dpaDimLHS * ceil(lhsEntriesPerMem / 1024) + dpaDimRHS * ceil(rhsEntriesPerMem / 1024))
     println("Resource predictions from cost model")
     println("=====================================")
     println(s"DPA LUT: $lut_array")
     println(s"DPA BRAM: $bram_array")
  }

  def headersAsList(): List[String] = {
    return List(
      "dpaLHS", "dpaRHS", "dpaCommon", "lhsMem", "rhsMem", "DRAM_rd", "DRAM_wr",
      "noShifter", "noNegate", "extraRegDPA", "extraRegDPU", "extraRegPC"
    )
  }

  def contentAsList(): List[String] = {
    return List(
      dpaDimLHS, dpaDimRHS, dpaDimCommon, lhsEntriesPerMem, rhsEntriesPerMem,
      mrp.dataWidth, mrp.dataWidth, noShifter,
      noNegate, extraRegs_DPA, extraRegs_DPU, extraRegs_PC
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
  val pcParams = new PopCountUnitParams(
    numInputBits = dpaDimCommon, extraPipelineRegs = extraRegs_PC
  )
  val dpuParams = new DotProductUnitParams(
    pcParams = pcParams, accWidth = accWidth, maxShiftSteps = maxShiftSteps,
    noShifter = noShifter, noNegate = noNegate, extraPipelineRegs = extraRegs_DPU
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
  Predef.assert(log2Up(lhsEntriesPerMem) <= BISMOLimits.inpBufAddrBits)
  Predef.assert(log2Up(rhsEntriesPerMem) <= BISMOLimits.inpBufAddrBits)
  Predef.assert(maxShiftSteps <= BISMOLimits.maxShift)
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

class BISMOInstructionRouter extends Module {
  val io = new Bundle {
    val in = Decoupled(UInt(width = BISMOLimits.instrBits)).flip
    val out_fetch = Decoupled(new BISMOFetchRunInstruction())
    val out_exec = Decoupled(new BISMOExecRunInstruction())
    val out_result = Decoupled(new BISMOResultRunInstruction())
  }
  val deintl = Module(new StreamDeinterleaver(
    numDests = 3, gen = io.in.bits,
    route = (x: UInt) => new BISMOInstruction().fromBits(x).targetStage
  )).io
  io.in <> deintl.in
  deintl.out.map(_.ready := Bool(false))
  // fetch
  io.out_fetch.valid := deintl.out(Stages.stgFetch).valid
  io.out_fetch.bits := io.out_fetch.bits.fromBits(deintl.out(Stages.stgFetch).bits.toBits)
  deintl.out(Stages.stgFetch).ready := io.out_fetch.ready
  // exec
  io.out_exec.valid := deintl.out(Stages.stgExec).valid
  io.out_exec.bits := io.out_exec.bits.fromBits(deintl.out(Stages.stgExec).bits.toBits)
  deintl.out(Stages.stgExec).ready := io.out_exec.ready
  // result
  io.out_result.valid := deintl.out(Stages.stgResult).valid
  io.out_result.bits := io.out_result.bits.fromBits(deintl.out(Stages.stgResult).bits.toBits)
  deintl.out(Stages.stgResult).ready := io.out_result.ready
  /*when(io.in.fire()) {
    printf("Routing new instruction, target stage = %d\n", new BISMOInstruction().fromBits(io.in.bits).targetStage)
  }*/
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
    // all instructions finished
    val prog_finished = Bool(OUTPUT)
    // instructions
    val op = Decoupled(UInt(width = BISMOLimits.instrBits)).flip
    // command counts in each queue
    val fetch_op_count = UInt(OUTPUT, width = 32)
    val exec_op_count = UInt(OUTPUT, width = 32)
    val result_op_count = UInt(OUTPUT, width = 32)
    // instantiated hardware config
    val hw = new BitSerialMatMulHWCfg(32).asOutput
    // performance counter I/O
    val perf = new BitSerialMatMulPerf(myP)
  }
  io.hw := myP.asHWCfgBundle(32)
  val opSwitch = Module(new BISMOInstructionRouter()).io
  // instantiate accelerator stages
  val fetchStage = Module(new FetchStage(myP.fetchStageParams)).io
  val execStage = Module(new ExecStage(myP.execStageParams)).io
  val resultStage = Module(new ResultStage(myP.resultStageParams)).io
  io.prog_finished := resultStage.prog_finished
  // instantiate the controllers for each stage
  val fetchCtrl = Module(new FetchController()).io
  val execCtrl = Module(new ExecController()).io
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

  // helper function to wire-up DecoupledIO to DecoupledIO with pulse generator
  def enqPulseGenFromValid[T <: Data](enq: DecoupledIO[T], vld: DecoupledIO[T]) = {
    enq.valid := vld.valid & !Reg(next=vld.valid)
    enq.bits := vld.bits
    vld.ready := enq.ready
  }

  // fill ocmInstrQ with instructions pushed via the CSRs
  val ocmInstrQ = Module(new FPGAQueue(io.op.bits, 2)).io
  enqPulseGenFromValid(ocmInstrQ.enq, io.op)

  // Fetch stage exposes instructions fetched from DRAM, but does not support
  // backpressure. even if the SW does not push too much instructions at once,
  // the upsizer here can cause data loss here since it's not fully pipelined.
  // solution: add a queue at the upsizer entry. how many elements?
  /*
  Assume we have 3x worth of instructions (x = cmdQueueEntries and 3 because we have 3 stages).
  Assume that the DDR channel width (64) is half the instruction width (128), using this as the base element.
  It'll normally take 6x cycles to write all the instructions in a FIFO with enough space.
  The upsizer will pop 2 basic elements (=1 instruction) every third cycle.
  So in the space of 6x cycles, it will pop 2*(6x/3) = 4x basic elements.
  This means the FIFO needs to have space for at least 6x - 4x = 2x basic elements.
  */
  // TODO generalize calculation for other DDRw/instrw ratios
  Predef.assert(myP.mrp.dataWidth * 2 == BISMOLimits.instrBits)
  val upsizerQueueElems = 2 * myP.cmdQueueEntries
  val instrResizeQ = Module(new FPGAQueue(UInt(width=myP.mrp.dataWidth), upsizerQueueElems)).io
  instrResizeQ.enq.valid := fetchStage.instrs.valid
  instrResizeQ.enq.bits := fetchStage.instrs.bits
  when(!instrResizeQ.enq.ready & instrResizeQ.enq.valid) {
    printf("ERROR: DRAM instruction queue could not receive data\n")
  }
  // StreamResizer to match instr q output width
  val instrResize = Module(new StreamResizer(
    inWidth = myP.mrp.dataWidth, outWidth = BISMOLimits.instrBits
  )).io
  instrResizeQ.deq <> instrResize.in

  // create mix of instructions from OCM and DRAM
  val instrMixer = Module(new StreamInterleaver(
    numSources = 2, gen = UInt(width = BISMOLimits.instrBits)
  )).io
  ocmInstrQ.deq <> instrMixer.in(0)
  //FPGAQueue(fetchStage.instrs, 2) <> instrMixer.in(1)
  FPGAQueue(instrResize.out, 2) <> instrMixer.in(1)

  // route incoming instructions according to target stage
  FPGAQueue(instrMixer.out, 2) <> opSwitch.in
  opSwitch.out_fetch <> fetchOpQ.enq
  opSwitch.out_exec <> execOpQ.enq
  opSwitch.out_result <> resultOpQ.enq

  /*
  when(opSwitch.in.fire()) {
    printf("opSwitch %x\n", opSwitch.in.bits)
  }
  when(ocmInstrQ.enq.fire()) {
    printf("OCM instrq %x\n", ocmInstrQ.enq.bits)
  }

  when(fetchStage.instrs.fire()) {
    printf("DRAM instrq %x\n", fetchStage.instrs.bits)
  }
  StreamMonitor(opSwitch.out_fetch, Bool(true), "fetch", true)
  StreamMonitor(opSwitch.out_exec, Bool(true), "exec", true)
  StreamMonitor(opSwitch.out_result, Bool(true), "res", true)
  StreamMonitor(instrMixer.in(1), Bool(true), "dram_ins", true)
  StreamMonitor(ocmInstrQ.enq, Bool(true), "ocm_ins", true)
  */

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
  fetchStage.start := fetchCtrl.start
  fetchCtrl.done := fetchStage.done
  fetchStage.csr := fetchCtrl.stageO
  // wire-up: exec controller and stage
  execStage.start := execCtrl.start
  execCtrl.done := execStage.done
  execStage.csr := execCtrl.stageO
  // wire-up: result controller and stage
  resultStage.start := resultCtrl.start
  resultCtrl.done := resultStage.done
  resultStage.csr := resultCtrl.stageO

  // wire-up: read channels to fetch stage
  fetchStage.dram.rd_req <> io.memPort(0).memRdReq
  io.memPort(0).memRdRsp <> fetchStage.dram.rd_rsp
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
  // wire-up: shared resource management (fetch and exec controllers)
  execCtrl.sync_out(0) <> syncFetchExec_free.enq
  syncFetchExec_free.deq <> fetchCtrl.sync_in(0)
  fetchCtrl.sync_out(0) <> syncFetchExec_filled.enq
  syncFetchExec_filled.deq <> execCtrl.sync_in(0)

  // wire-up: shared resource management (exec and result stages)
  resultCtrl.sync_out(0) <> syncExecResult_free.enq
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
  // wire-up: write channels from result stage
  resultStage.dram.wr_req <> io.memPort(0).memWrReq
  resultStage.dram.wr_dat <> io.memPort(0).memWrDat
  io.memPort(0).memWrRsp <> resultStage.dram.wr_rsp

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
