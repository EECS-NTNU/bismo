package bismo

import Chisel._

// wraps the FPGA-optimized VHDL compressor generator originally developed by
// Thomas Preusser. although the generator supports multi-bit operands, we only
// use it for regular binary operands here.

class BlackBoxCompressorParams(
  val N: Int, // bitwidth of compressor inputs
  val D: Int  // number of pipeline registers, subject to
              // compressor tree depth
) extends PrintableParam {
  def headersAsList(): List[String] = {
    return List("Dk", "BBCompressorLatency")
  }

  def contentAsList(): List[String] = {
    return List(N,D).map(_.toString)
  }
  // TODO allow specifying D = -1 to pick a known-good pipelining depth for
  // the given N, and return the latency with .getLatency function
}

// Chisel Module wrapper around generated compressor
class BlackBoxCompressor(p : BlackBoxCompressorParams) extends Module {
	def outputbits = log2Up(p.N) + 1
	val io = new Bundle{
		val c = Bits(INPUT, width = p.N)
		val d = Bits(INPUT, width = p.N)
		val r = Bits(OUTPUT, width = outputbits)
	}
	val inst = Module(new mac(BB_WA = outputbits, BB_N = p.N, BB_WD = 1, BB_WC = 1, BB_D = p.D)).io
	inst.a := UInt(0)
	inst <> io
}

// actual BlackBox that instantiates the VHDL unit
class mac (
  BB_WA : Int,  // result precision
  BB_N: Int,    // number of elements in dot product
  BB_WD : Int,  // input operand 1 precision
  BB_WC : Int,  // input operand 2 precision
  BB_D : Int    // optional pipeline regs to add
) extends BlackBox {
	val io = new  Bundle {
    // accumulator input is unused
		val a = Bits(INPUT, width=BB_WA)
    // c and d are the inputs to the binary dot product
		val c = Bits(INPUT, width=BB_N*BB_WC)
		val d = Bits(INPUT, width=BB_N*BB_WD)
    // r contains the result after D cycles
		val r = Bits(OUTPUT, width=BB_WA)
		a.setName("a")
		c.setName("c")
		d.setName("d")
		r.setName("r")
	}
	setVerilogParameters( new VerilogParameters {
		val WA 	  :Int = BB_WA
		val N 	  :Int = BB_N
		val WD    :Int = BB_WD
		val WC    :Int = BB_WC
		val DEPTH :Int = BB_D
	})

  // clock needs to be added manually to BlackBox
	addClock(Driver.implicitClock)

	// Behavioral model for compressor: delayed AND-popcount
	if (BB_WD == 1 && BB_WC == 1) {
		io.r := ShiftRegister(PopCount(io.c & io.d), BB_D)
	}
}
