// Author:  Davide Conficconi
// Date: 25/09/2018
// Revision: 0
package bismo

import Chisel._
import fpgatidbits.ocm._
import fpgatidbits.streams._


// TODO: Brainstorm on the DL for the next stage: I write K (elem bit-width) times
// TODO: with M (matrix rows) parallel requests with bit-width of 1b times N (matrix columns)
// TODO Should I care about how many bits of the same row read next stage? then a temporary buffer is needed

class Parallel2BSStageParams(
    // building block params
    val suParams : SerializerUnitParams,
    // threshold memory depth (how many entries, address space)
    val thMemDepth : Int = 8,
    val bsMemDepth : Int = 8,
    val inputMemAddr : Int ,
    val resMemAddr : Int,
    val thMemLatency : Int = 1,
    // levels of registers before (on address input) and after (on data output)
    // of each tile memory BRAM
    val bramInRegs: Int = 1,
    val bramOutRegs: Int = 1
    ) extends PrintableParam {

  // latency
  val inMemLatency: Int = bramInRegs


  def getRows(): Int = {
    return suParams.matrixRows
  }
  def getInBits() : Int = {
    return suParams.inPrecision
  }

  def getCols() : Int = {
    return suParams.matrixCols
  }

  def headersAsList(): List[String] = {
    return suParams.headersAsList() ++  List()
  }

  def contentAsList(): List[String] = {
    return suParams.contentAsList() ++ List().map(_.toString)
  }

}


// interface to hardware config available to software
class Parallel2BSStageCfgIO() extends Bundle {
  val config_inPrecision = UInt(OUTPUT, width = 32)
  val config_staticCounter = Bool(OUTPUT)
  //TODO NOT REALLY USED NOW
  override def cloneType: this.type =
    new Parallel2BSStageCfgIO().asInstanceOf[this.type]
}

// interface towards controller for the execute stage
class Parallel2BSStageCtrlIO(myP: Parallel2BSStageParams) extends PrintableBundle {

  //TODO NOT REALLY USED NOW
  val resOffset = UInt(width = 32) // start offset for result memory
  val thrOffset = UInt(width = 32) // start offset for threshold memory
  // TODO Design time fixed width?
  val count_bits_shifting = UInt(width = 32)
  // write to result memory at the end of current execution
  val writeEn = Bool()
  // result memory address to use for writing
  val writeAddr = UInt(width = log2Up(myP.thMemDepth))

  override def cloneType: this.type =
    new Parallel2BSStageCtrlIO(myP).asInstanceOf[this.type]

  val printfStr = "(offs lhs/rhs = %d/%d, ntiles = %d, << %d, w? %d/%d)\n"
  val printfElems = {() =>  Seq(
    //lhsOffset, rhsOffset, numTiles, shiftAmount, writeEn, writeAddr
  )}
}


// interface towards tile memories (LHS(Thresholds)/RHS(Activation) BRAMs)
class Parallel2BSStageTileMemIO(myP: Parallel2BSStageParams) extends Bundle {
  val thr_req = Vec.fill(myP.getRows()) { Vec.fill(myP.getCols()){
    new OCMRequest(myP.getInBits(), log2Up(myP.thMemDepth)).asOutput
  }}
  val thr_rsp = Vec.fill(myP.getRows()) { Vec.fill(myP.getCols()){
    new OCMResponse(myP.getInBits() ).asInput
  }}

  override def cloneType: this.type =
    new Parallel2BSStageTileMemIO(myP).asInstanceOf[this.type]
}

//TODO Assumed able to write one row per request and not in bit serial way
// interface towards result stage
class Parallel2BSStageResMemIO(myP: Parallel2BSStageParams) extends Bundle {
  val req = Vec.fill(myP.getRows()) { new OCMRequest(
      myP.getCols(), log2Up(myP.resMemAddr)
    ).asOutput
  }

  override def cloneType: this.type =
    new Parallel2BSStageResMemIO(myP).asInstanceOf[this.type]
}

class Parallel2BSStage(val myP: Parallel2BSStageParams) extends Module{
  val io = new Bundle{
    val start = Bool(INPUT) // hold high while running
    val done = Bool(OUTPUT) // high when done until start=0
    val cfg = new Parallel2BSStageCfgIO()
    val ctrl = new Parallel2BSStageCtrlIO(myP).asInput
    val res = new Parallel2BSStageResMemIO(myP)
    val inMemory = new Parallel2BSStageTileMemIO(myP)
  }


  val serunit = Module(new SerializerUnit(myP.suParams)).io

  for(i <- 0 until myP.getRows())
    for(j <- 0 until myP.getCols()){
      io.inMemory.thr_req(i)(j).writeData := UInt(0)
      io.inMemory.thr_req(i)(j).writeEn := Bool(false)
      //TODO tiling not handled
      io.inMemory.thr_req(i)(j).addr := UInt(0) + io.ctrl.thrOffset
      //response handling
      //ASSUMPTION: same dimension matrix fetching - matrix serializer
      serunit.input(i)(j):=io.inMemory.thr_rsp(i)(j).readData
    }

  serunit.counterValue := io.ctrl.count_bits_shifting
  serunit.start := ShiftRegister(io.start, myP.inMemLatency)


  /*Ogghei, la mia SU produce output validi ogni ciclo, cosa piu interessante da fare e registro e spararli fuori
  il  valid della SU dice che ho mandato in output tutti i bit e ora c'e l'ultimo.
  il redy glielo devo dare quando finisco e voglio riniziare? o solo per reiniziallizzare cos?

*/
  val seqgen = Module(new SequenceGenerator(log2Up(myP.bsMemDepth))).io

  seqgen.start := ShiftRegister(io.start, myP.inMemLatency)
  seqgen.init := UInt(0)
  seqgen.step := UInt(1)
  seqgen.count := io.ctrl.count_bits_shifting
  //Useful for multiple bits outputs
  seqgen.seq.ready := Bool(true)

  //TODO care on width for future digit serialization
  val intermediate_buffer = Vec.fill(myP.getRows()){ Vec.fill(myP.getCols()) {Reg(init = UInt(0,width = 1)) } }

  for(i<- 0 until myP.getRows())
    for(j <- 0 until myP.getCols()){
      intermediate_buffer(i)(j) :=  serunit.out.bits(i)(j)
    }

  for(i <- 0 until myP.getRows()){
    // Sequence generator?
    io.res.req(i).addr := UInt(myP.resMemAddr) + io.ctrl.resOffset + seqgen.seq.bits
    //TODO is it enough?
    io.res.req(i).writeEn := seqgen.seq.valid
    for(j <- 0 until myP.getCols()){
      io.res.req(i).writeData := Cat(intermediate_buffer(i)(j))
    }
  }

  io.done := serunit.out.valid
  when(serunit.out.valid && !io.start){
    serunit.out.ready := Bool(true)
  }.otherwise{
    serunit.out.ready := Bool(false)
  }
  /*


  val start_r = Reg(init = false.B, next = io.start & seqgen.seq.valid)
  //TODO this is for a single run per start
  val start_pulse = io.start & seqgen.seq.valid & !start_r | ( start_r & !seqgen.seq.valid  )
  //TODO this is for a continuous run per start
  //val start_pulse = io.start & !start_r

  //ASSUMING that my data are available as long as I need
  for(i <- 0 until myP.getUnrollRows()) {
    for (j <- 0 until myP.getUnrollCols()) {
      thu.inputMatrix.bits.i(i)(j) := io.inMemory.act_rsp(i)(j).readData//(myP.getInBits() * (1 + j) - 1, myP.getInBits() * j)
      /*when(start_pulse){
        printf("[HW] Matrix elem %d, %d Read elem from Unit: %d\n",UInt(i), UInt(j), io.inMemory.act_rsp(i)(j).readData)//(myP.getInBits() * (1 + j) - 1, myP.getInBits() * j))
      }*/
    }
    for(j <- 0 until myP.getThUnroll())
      thu.thInterf.thresholdData(i)(j) := io.inMemory.thr_rsp(i)(j).readData//( myP.getInBits()*(1+j) - 1, myP.getInBits()*j)
  }


  thu.inputMatrix.valid := ShiftRegister(start_pulse, myP.activationMemoryLatency-1)

  //time to write is the latency of the unit + time to write all the outputs
  val time_to_write = myP.thuParams.getLatency()
  //TODO: this is time to respond for the Data BRAM
  val end = ShiftRegister(thu.outputMatrix.valid, time_to_write)
  val end_r = Reg(init = false.B, next = end)
  val end_pulse = end & !end_r
  io.done := (end_pulse | end_r)
  val i = Reg(init = UInt(0, width = 32))
  /*when(end){
    printf("[HW] It takes %d cycles to restart!!!\n",i);
    i := i + UInt(1)
  }*/
  thu.outputMatrix.ready := end & !io.start
  for(i <- 0 until myP.getUnrollRows())
    for(j <- 0 until myP.getUnrollCols()){
      io.res.req(i)(j).writeEn := thu.outputMatrix.valid
      io.res.req(i)(j).addr := UInt(0)
      io.res.req(i)(j).writeData := thu.outputMatrix.bits.o(i)(j)//UInt(0, width = myP.getResBitWidth() * myP.getCols()) | (thu.outputMatrix.bits.o(i)(j) << j)
    }
*/
}