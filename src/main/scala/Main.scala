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
// Modifications: Davide Conficconi for New components Characterization and Cosimulation

package bismo

import Chisel._
import sys.process._
import fpgatidbits.PlatformWrapper._
import fpgatidbits.TidbitsMakeUtils

// Main entry points into the different functionalities provided by this repo.
// There are four different entry points:
// - ChiselMain to generate Verilog
// - EmuLibMain to generate HW+SW cosimulations
// - DriverMain to generate register driver code for BISMO
// - CharacterizeMain to characterize FPGA resource usage of components

object Settings {
  type AccelInstFxn = PlatformWrapperParams ⇒ GenericAccelerator
  type AccelMap = Map[String, AccelInstFxn]
  val myInstParams = new BitSerialMatMulParams(
    dpaDimLHS = 8, dpaDimRHS = 8, dpaDimCommon = 256,
    lhsEntriesPerMem = 64 * 32 * 64 / (8 * 256),
    rhsEntriesPerMem = 64 * 32 * 64 / (8 * 256),
    mrp = PYNQZ1Params.toMemReqParams(),
    cmdQueueEntries = 256,
    thrEntriesPerMem = 64, maxQuantDim = 4, quantFolding = 1
  )
  val myInstFxn: AccelInstFxn = {
    (p: PlatformWrapperParams) ⇒ new BitSerialMatMulAccel(myInstParams, p)
  }

  val myInstP2S: AccelInstFxn = {
    (p: PlatformWrapperParams) ⇒ new EmuTestP2SAccel(8, 8, 64, true, p)
  }

  def makeInstFxn(myP: BitSerialMatMulParams): AccelInstFxn = {
    return { (p: PlatformWrapperParams) ⇒ new BitSerialMatMulAccel(myP, p) }
  }

  def makeInstFxn(m: Int, n: Int, o: Int, fast: Boolean): AccelInstFxn = {
    return { (p: PlatformWrapperParams) ⇒ new EmuTestP2SAccel(m, n, o, fast, p) }
  }
  // instantiate smaller accelerator for emu for faster testing
  val emuInstParams = new BitSerialMatMulParams(
    dpaDimLHS = 2, dpaDimRHS = 2, dpaDimCommon = 128, lhsEntriesPerMem = 128,
    rhsEntriesPerMem = 128, mrp = PYNQZ1Params.toMemReqParams())

  // given accelerator or hw-sw-test name, return its hardware instantiator
  val emuP = TesterWrapperParams
  val emuMap: AccelMap = Map(
    // "main" is the emulator for the default target
    "main" -> { p ⇒ new BitSerialMatMulAccel(emuInstParams, emuP) },
    // HW-SW cosimulation tests
    // for these tests (EmuTest*) the same name is assumed to be the cpp file
    // that defines the software part of the test under test/cosim
    "EmuTestExecStage" -> {p => new EmuTestExecStage(emuP)},
    "EmuTestFetchStage" -> {p => new EmuTestFetchStage(2, 2, emuP)},
    "EmuTestResultStage" -> {p => new EmuTestResultStage(2, emuP)},
    "EmuTestThrStage" -> {p => new EmuTestThrStage(mRows = 2, mCols = 2, inBits = 32,outBits =  4, thUnroll= 15, emuP)},
    "EmuTestP2BSStage" -> {p => new EmuTestP2BSStage(2,3,4,1,16,emuP)},
    "EmuTestP2SAccel" -> { p ⇒ new EmuTestP2SAccel(8, 8, 64, true, emuP) })
}

// call this object's main method to generate Chisel Verilog
object ChiselMain {
  def main(args: Array[String]): Unit = {
    val platformName: String = args(0)
    val targetDir: String = args(1)
    val dpaDimLHS: Int = args(2).toInt
    val dpaDimCommon: Int = args(3).toInt
    val dpaDimRHS: Int = args(4).toInt
    val accInst = Settings.makeInstFxn(
      //TODO updated for BOB :)
      new BitSerialMatMulParams(
        dpaDimLHS = dpaDimLHS, dpaDimRHS = dpaDimRHS, dpaDimCommon = dpaDimCommon,
        lhsEntriesPerMem = 64 * 32 * 64 / (dpaDimLHS * dpaDimCommon),
        rhsEntriesPerMem = 64 * 32 * 64 / (dpaDimRHS * dpaDimCommon),
        thrEntriesPerMem = 64, maxQuantDim = 4, quantFolding = 15,
        mrp = PYNQZ1Params.toMemReqParams()
      )
    )
    val platformInst = TidbitsMakeUtils.platformMap(platformName)

    val chiselArgs = Array("--backend", "v", "--targetDir", targetDir)
    chiselMain(chiselArgs, () ⇒ Module(platformInst(accInst)))

  }
}

// call this object's main method to generate Chisel Verilog for the P2S
object P2SMain {
  def main(args: Array[String]): Unit = {
    val platformName: String = args(0)
    val targetDir: String = args(1)
    val maxInBw: Int = args(2).toInt
    val nInElemPerWord: Int = args(3).toInt
    val outStreamSize: Int = args(4).toInt
    val fast: Boolean = args(5).toBoolean

    val accInst = Settings.makeInstFxn(
      m = maxInBw, n = nInElemPerWord, o = outStreamSize, fast = fast)
    val platformInst = TidbitsMakeUtils.platformMap(platformName)

    val chiselArgs = Array("--backend", "v", "--targetDir", targetDir)
    chiselMain(chiselArgs, () ⇒ Module(platformInst(accInst)))

  }
}

object ResModelMain {
  def main(args: Array[String]): Unit = {
    val platformName: String = args(0)
    val targetDir: String = args(1)
    val dpaDimLHS: Int = args(2).toInt
    val dpaDimCommon: Int = args(3).toInt
    val dpaDimRHS: Int = args(4).toInt
    val params = new BitSerialMatMulParams(
      dpaDimLHS = dpaDimLHS, dpaDimRHS = dpaDimRHS, dpaDimCommon = dpaDimCommon,
      lhsEntriesPerMem = 1024,
      rhsEntriesPerMem = 1024,
      mrp = PYNQZ1Params.toMemReqParams())
    params.estimateResources()
  }
}

// call this object's main method to generate a C++ static library containing
// the cycle-accurate emulation model for the chosen accelerator. the interface
// of the model is compatible with  the fpgatidbits.PlatformWrapper hw/sw
// interface.
object EmuLibMain {
  def main(args: Array[String]): Unit = {
    val emuName: String = args(0)
    val emuDir: String = args(1)
    val debugMode: Int = args(2).toInt
    val accInst: Settings.AccelInstFxn = Settings.emuMap(emuName)
    if (debugMode == 1) {
      TidbitsMakeUtils.makeEmulatorLibrary(
        accInst, emuDir, Seq("--std=c++11", "-DDEBUG"), Seq("--vcd"))
    } else {
      TidbitsMakeUtils.makeEmulatorLibrary(accInst, emuDir, Seq("--std=c++11"))
    }
  }
}

// call this object's Main to generate Verilog and characterize it in terms of
// the hardware resources required and achievable Fmax
object CharacterizeMain {
  def makeParamSpace_DPA(): Seq[DotProductArrayParams] = {
    val spatial_dim = 1 to 8
    val popcount_dim = for (i ← 4 to 8) yield (1 << i)
    val ret = for {
      m ← spatial_dim
      n ← spatial_dim
      k ← popcount_dim
    } yield new DotProductArrayParams(
      m = m, n = n, dpuParams = new DotProductUnitParams(
      maxShiftSteps = 16, accWidth = 32, pcParams = new PopCountUnitParams(
      numInputBits = k)))
    // (m, n) resource-wise equivalent, so keep half the cases to avoid duplicates
    return ret.filter(x ⇒ (x.m >= x.n))
  }
  val instFxn_DPA = { p: DotProductArrayParams ⇒ Module(new DotProductArray(p)) }

  def makeParamSpace_PC(): Seq[PopCountUnitParams] = {
    return for {
      extra_regs ← 0 to 1
      i ← for (i ← 5 to 10) yield (1 << i)
    } yield new PopCountUnitParams(
      numInputBits = i, extraPipelineRegs = extra_regs)
  }
  val instFxn_PC = { p: PopCountUnitParams ⇒ Module(new PopCountUnit(p)) }

  def makeParamSpace_DPU(): Seq[DotProductUnitParams] = {
    val noshift = Seq(false, true)
    val noneg = Seq(false, true)
    return for {
      popc ← for (i ← 5 to 10) yield (1 << i)
      ns ← noshift
      nn ← noneg
      extra_regs_pc ← 0 to 1
      extra_regs_dpu ← 0 to 1
    } yield new DotProductUnitParams(
      pcParams = new PopCountUnitParams(
        numInputBits = popc, extraPipelineRegs = extra_regs_pc),
      noShifter = ns, noNegate = nn, accWidth = 32, maxShiftSteps = 16,
      extraPipelineRegs = extra_regs_dpu)
  }
  val instFxn_DPU = { p: DotProductUnitParams ⇒ Module(new DotProductUnit(p)) }

  def makeParamSpace_Main(): Seq[BitSerialMatMulParams] = {
    return for {
      lhs ← for (i ← 1 to 5) yield (2 * i)
      rhs ← for (i ← 1 to 5) yield (2 * i)
      z ← 64 to 64
      lmem ← 1024 to 1024
      rmem ← 1024 to 1024
    } yield new BitSerialMatMulParams(
      dpaDimLHS = lhs, dpaDimRHS = rhs, dpaDimCommon = z,
      lhsEntriesPerMem = lmem, rhsEntriesPerMem = rmem,
      mrp = PYNQZ1Params.toMemReqParams())
  }
  val instFxn_Main = { p: BitSerialMatMulParams ⇒ Module(new BitSerialMatMulAccel(p, PYNQZ1Params)) }

  def makeParamSpace_ResultStage(): Seq[ResultStageParams] = {
    return for {
      wc ← 1 to 1
      sd ← for (i ← 1 to 5) yield (2 * i)
    } yield new ResultStageParams(
      accWidth = 32, dpa_rhs = sd, dpa_lhs = sd,
      mrp = PYNQZ1Params.toMemReqParams(), resMemReadLatency = 0)
  }
  val instFxn_ResultStage = { p: ResultStageParams ⇒ Module(new ResultStage(p)) }

  def makeParamSpace_ResultBuf(): Seq[ResultBufParams] = {
    return for {
      a ← 1 to 4
      d ← Seq(16, 32, 64)
      r ← Seq(0, 1)
    } yield new ResultBufParams(
      addrBits = a, dataBits = d, regIn = r, regOut = r)
  }
  val instFxn_ResultBuf = { p: ResultBufParams ⇒ Module(new ResultBuf(p)) }

  def makeParamSpace_FetchStage(): Seq[FetchStageParams] = {
    return for {
      c ← 1 to 4
      n ← for (i ← 1 to 8) yield c * i
    } yield new FetchStageParams(
      numLHSMems = n, numRHSMems = n,
      numAddrBits = 10, mrp = PYNQZ1Params.toMemReqParams())
  }
  val instFxn_FetchStage = { p: FetchStageParams ⇒ Module(new FetchStage(p)) }

    def makeParamSpace_THU(): Seq[ThresholdingUnitParams] = {
    return for {
      inP <- 4 to 4
      mOutP <- 1 to 1
      rows <- 2 to 2
      cols <- 2 to 2
      unrollBB <- 1 to 1
      unRows <- 2 to 2
      unCols <- 2 to 2
    } yield new ThresholdingUnitParams(
      thBBParams = new ThresholdingBuildingBlockParams( inPrecision = inP, popcountUnroll = unrollBB,  outPrecision = mOutP),
      inputBitPrecision = inP, maxOutputBitPrecision = mOutP, matrixRows = rows,
      matrixColumns = cols, unrollingFactorOutputPrecision = unrollBB,
      unrollingFactorRows = unRows, unrollingFactorColumns = unCols
    )
  }

  val instFxn_THU = {p: ThresholdingUnitParams => Module(new ThresholdingUnit(p))}


  def makeParamSpace_TBB(): Seq[ThresholdingBuildingBlockParams] = {
    return for {
      m <- 32 to 32
      k <- 1 to 1
      n <- 1 to 1
    } yield new ThresholdingBuildingBlockParams(
      inPrecision = m, popcountUnroll = k, outPrecision = n
    )
  }
  val instFxn_TBB = {p: ThresholdingBuildingBlockParams => Module(new ThresholdingBuildingBlock(p))}

  def makeParamSpace_thrStage(): Seq[ThrStageParams] = {
    return for {
      inP <- Seq(4, 8, 16, 32)
      mOutP <- Seq(1,2,3,4,8)
      rows <- Seq(1,2,3,4,6,8)
      cols <- Seq(1,2,3,4,6,8,32,64)
      unrollBB <- Seq(1,scala.math.pow(2,mOutP).toInt - 1)
      //unRows <- 8 to 8
      //unCols <- 8 to 8
      resAddr <- 8 to 8
      inAddr <- 8  to 8
      thAddr <- 8 to 8
    } yield new ThrStageParams(
     thresholdMemDepth = thAddr, inputMemDepth = inAddr, resMemDepth = resAddr,
      thuParams = new ThresholdingUnitParams(
        thBBParams = new ThresholdingBuildingBlockParams( inPrecision = inP, popcountUnroll = unrollBB,  outPrecision = mOutP),
        inputBitPrecision = inP, maxOutputBitPrecision = mOutP, matrixRows = rows,
        matrixColumns = cols, unrollingFactorOutputPrecision = unrollBB,
        unrollingFactorRows = rows, unrollingFactorColumns = cols
      )
    )
  }

val instFxn_thrStage = {p: ThrStageParams => Module(new ThrStage(p))}

  def makeParamSpace_BlackBoxCompressor(): Seq[BlackBoxCompressorParams] = {
    return for {
      n ← for (i ← 6 to 8) yield 1 << i
      d ← 0 to 0
      aw <- Seq(2, 3)// 2, 4, 8, 16)
      bw <- Seq(1, 2, 3)// 2, 4, 8, 16)
    } yield new BlackBoxCompressorParams(
      N = n, D = d, WD = aw, WC = bw)
  }

  val instFxn_BlackBoxCompressor = { p: BlackBoxCompressorParams ⇒ Module(new BlackBoxCompressor(p)) }

  val instFxn_BBC = { p: BlackBoxCompressorParams ⇒ Module(new CharacterizationBBCompressor(p)) }

  def makeParamSpace_P2SAccel(): Seq[StandAloneP2SParams] = {
    return for {
      mbw ← Seq(4, 8, 16, 32, 64)
      nxw ← Seq(64 / mbw, 128 / mbw)
      fast ← Seq(false, true)
    } yield new StandAloneP2SParams(
      maxInBw = mbw, nInElemPerWord = nxw, outStreamSize = mbw * nxw, mrp = PYNQZ1Params.toMemReqParams(),
      fastMode = fast)
  }
  val instFxn_P2SAccel = { p: StandAloneP2SParams ⇒ Module(new StandAloneP2SAccel(p, TesterWrapperParams)) }

  def makeParamSpace_SU(): Seq[SerializerUnitParams] = {
    return for {

      inBW <- Seq(4, 8, 16, 32)
      maxCounterBW <- Seq(inBW, 32)
      rows <- 8 to 8
      cols <- 8 to 8
      static <- Seq(true,false)

    } yield new SerializerUnitParams(
    inPrecision = inBW, matrixRows = rows, matrixCols = cols, staticCounter = static, maxCounterPrec = maxCounterBW
    )
  }

  val instFxn_SU = {p: SerializerUnitParams => Module(new SerializerUnit(p))}

  def makeParamSpace_P2BSStage(): Seq[Parallel2BSStageParams] = {
    return for {
      inBW <- Seq(4, 8, 16, 32)
      maxCounterBW <- Seq(inBW, 32)
      rows <- Seq(2,3,4,8)
      cols <- Seq(2,3,4,8)
      static <- Seq(true,false)
      inMemDepth <- 8 to 8
      resMemDepth <- Seq(inBW)
      memAddr <- 0 to 0
      regLatency <- 1 to 1


    } yield new Parallel2BSStageParams(
      suParams = new SerializerUnitParams ( inPrecision = inBW, matrixRows = rows, matrixCols = cols, staticCounter = static, maxCounterPrec = maxCounterBW),
      thMemDepth  = inMemDepth, bsMemDepth = resMemDepth,
      thMemLatency = regLatency, bramInRegs= regLatency, bramOutRegs = regLatency
    )
  }

  val instFxn_P2BSStage = {p: Parallel2BSStageParams => Module(new Parallel2BSStage(p))}


  def makeParamSpace_SQUAT(): Seq[BitSerialMatMulQuantParams] = {
    return for {
      lhs <- for(i <- 1 to 8) yield (2*i)
      rhs <- for(i <- 1 to 8) yield (2*i)
      z <- 64 to 64
      lmem <- 512 to 512
      rmem <- 512 to 512
      thmem <- 512 to 512
      maxquantDim <- Seq(2,4,8)
      thFolding <- Seq(1,scala.math.pow(2,maxquantDim).toInt - 1)
    } yield new BitSerialMatMulQuantParams(
      dpaDimLHS = lhs, dpaDimRHS = rhs, dpaDimCommon = z,
      lhsEntriesPerMem = lmem, rhsEntriesPerMem = rmem,
      mrp = PYNQZ1Params.toMemReqParams(),
      thrEntriesPerMem = thmem, maxQuantDim = maxquantDim, quantFolding = thFolding

    )
  }
  val instFxn_SQUAT = {p: BitSerialMatMulQuantParams => Module(new BitSerialMatMulQuantAccel(p, PYNQZ1Params))}

  def makeParamSpace_BOB(): Seq[BOBParams] = {
    return for {
      lhs <- for(i <- 1 to 5) yield (2*i)
      rhs <- for(i <- 1 to 5) yield (2*i)
      z <- 64 to 64
      lmem <- 512 to 512
      rmem <- 512 to 512
      thmem <- 512 to 512
      maxquantDim <- Seq(1,2,4)
      thFolding <- Seq(1,scala.math.pow(2,maxquantDim).toInt - 1)
    } yield new BOBParams(
      dpaDimLHS = lhs, dpaDimRHS = rhs, dpaDimCommon = z,
      lhsEntriesPerMem = lmem, rhsEntriesPerMem = rmem,
      mrp = PYNQZ1Params.toMemReqParams(),
      thrEntriesPerMem = thmem, maxQuantDim = maxquantDim, quantFolding = thFolding

    )
  }
  val instFxn_BOB = {p: BOBParams => Module(new BOBAccel(p, PYNQZ1Params))}

  def main(args: Array[String]): Unit = {
    val chName: String = args(0)
    val chPath: String = args(1)
    val platform: String = args(2)
    val chLog = chName + ".log"
    val fpgaPart: String = VivadoSynth.fpgaPartMap(platform)

    if (chName == "CharacterizePC") {
      VivadoSynth.characterizeSpace(makeParamSpace_PC(), instFxn_PC, chPath, chLog, fpgaPart)
    } else if (chName == "CharacterizeDPU") {
      VivadoSynth.characterizeSpace(makeParamSpace_DPU(), instFxn_DPU, chPath, chLog, fpgaPart)
    } else if (chName == "CharacterizeDPA") {
      VivadoSynth.characterizeSpace(makeParamSpace_DPA(), instFxn_DPA, chPath, chLog, fpgaPart)
    } else if (chName == "CharacterizeMain") {
      VivadoSynth.characterizeSpace(makeParamSpace_Main(), instFxn_Main, chPath, chLog, fpgaPart)
    } else if (chName == "CharacterizeResultStage") {
      VivadoSynth.characterizeSpace(makeParamSpace_ResultStage(), instFxn_ResultStage, chPath, chLog, fpgaPart)
    } else if (chName == "CharacterizeResultBuf") {
      VivadoSynth.characterizeSpace(makeParamSpace_ResultBuf(), instFxn_ResultBuf, chPath, chLog, fpgaPart)
    } else if (chName == "CharacterizeFetchStage") {
      VivadoSynth.characterizeSpace(makeParamSpace_FetchStage(), instFxn_FetchStage, chPath, chLog, fpgaPart)
    } else if (chName == "CharacterizeTHU") {
      VivadoSynth.characterizeSpace(makeParamSpace_THU(), instFxn_THU, chPath, chLog, fpgaPart)
    } else if (chName == "CharacterizeTBB") {
      VivadoSynth.characterizeSpace(makeParamSpace_TBB(), instFxn_TBB, chPath, chLog, fpgaPart)
    } else if (chName == "CharacterizeThrStage") {
      VivadoSynth.characterizeSpace(makeParamSpace_thrStage(), instFxn_thrStage, chPath, chLog, fpgaPart)
    } else if (chName == "CharacterizeSU") {
      VivadoSynth.characterizeSpace(makeParamSpace_SU(), instFxn_SU, chPath, chLog, fpgaPart)
    } else if (chName == "CharacterizeP2BS") {
      VivadoSynth.characterizeSpace(makeParamSpace_P2BSStage(), instFxn_P2BSStage, chPath, chLog, fpgaPart)
    } else if (chName == "CharacterizeSQUAT") {
      VivadoSynth.characterizeSpace(makeParamSpace_SQUAT(), instFxn_SQUAT, chPath, chLog, fpgaPart)
    } else if (chName == "CharacterizeBOB") {
      VivadoSynth.characterizeSpace(makeParamSpace_BOB(), instFxn_BOB, chPath, chLog, fpgaPart)
    } else if (chName == "CharacterizeBBCompressor") {
      VivadoSynth.characterizeSpace(makeParamSpace_BlackBoxCompressor(), instFxn_BlackBoxCompressor, chPath, chLog, fpgaPart)
    }else if (chName == "CharacterizeBBC"){
      VivadoSynth.characterizeSpace(makeParamSpace_BlackBoxCompressor(), instFxn_BBC, chPath, chLog, fpgaPart)
    } else if (chName == "CharacterizeP2SAccel") {
      VivadoSynth.characterizeSpace(makeParamSpace_P2SAccel(), instFxn_P2SAccel, chPath, chLog, fpgaPart)
    } else {
      println("Unrecognized target for characterization")
    }
  }
}

// call this object's main method to generate the register driver for your
// accelerator for PYNQ. expects the following command line arguments, in order:
// 1. name of platform (must be supported by fpga-tidbits PlatformWrapper)
// 2. path to output directory for generated files
object DriverMain {
  // utility functions to copy files inside Scala
  def fileCopy(from: String, to: String) = {
    s"cp -f $from $to" !
  }

  def fileCopyBulk(fromDir: String, toDir: String, fileNames: Seq[String]) = {
    for (f ← fileNames)
      fileCopy(s"$fromDir/$f", s"$toDir/$f")
  }

  def main(args: Array[String]): Unit = {
    val platformName: String = args(0)
    val targetDir: String = args(1)
    val drvSrcDir: String = args(2)
    // instantiate the wrapper accelerator
    val accInst = Settings.myInstFxn
    val platformInst = TidbitsMakeUtils.platformMap(platformName)
    val myModule = Module(platformInst(accInst))
    // generate the register driver
    myModule.generateRegDriver(targetDir)
    // copy additional driver files
    fileCopyBulk(drvSrcDir, targetDir, myModule.platformDriverFiles)
  }
}

// call this object's main method to generate the register driver for your
// accelerator for PYNQ. expects the following command line arguments, in order:
// 1. name of platform (must be supported by fpga-tidbits PlatformWrapper)
// 2. path to output directory for generated files
object P2SDriverMain {
  // utility functions to copy files inside Scala
  def fileCopy(from: String, to: String) = {
    s"cp -f $from $to" !
  }

  def fileCopyBulk(fromDir: String, toDir: String, fileNames: Seq[String]) = {
    for (f ← fileNames)
      fileCopy(s"$fromDir/$f", s"$toDir/$f")
  }

  def main(args: Array[String]): Unit = {
    val platformName: String = args(0)
    val targetDir: String = args(1)
    val drvSrcDir: String = args(2)
    // instantiate the wrapper accelerator
    val accInst = Settings.myInstP2S
    val platformInst = TidbitsMakeUtils.platformMap(platformName)
    val myModule = Module(platformInst(accInst))
    // generate the register driver
    myModule.generateRegDriver(targetDir)
    // copy additional driver files
    fileCopyBulk(drvSrcDir, targetDir, myModule.platformDriverFiles)
  }
}
