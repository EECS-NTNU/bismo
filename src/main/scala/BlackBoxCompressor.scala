package bismo

import Chisel._


class BlackBoxCompressorParams(
  val N: Int,
  val D: Int
) extends PrintableParam {
  def headersAsList(): List[String] = {
    return List("Dk","Depth of the pipeline")
  }

  def contentAsList(): List[String] = {
    return List(N,D).map(_.toString)
  }
}
class BlackBoxCompressor(P : BlackBoxCompressorParams) extends Module {
	def outputbits = log2Up(P.N) + 1
	val io = new Bundle{
		val c = Bits(INPUT, width = P.N)
		val d = Bits(INPUT, width = P.N)
		val r = Bits(OUTPUT, width = outputbits)
	}
	val inst = Module(new mac(BB_WA = outputbits, BB_N = P.N, BB_WD = 1, BB_WC = 1, BB_D = P.D)).io
	inst.a := UInt(0)
	addResetPin(Driver.implicitReset)
	inst <> io 
	inst.rst := Driver.implicitReset
}

class mac (BB_WA : Int, BB_N: Int, BB_WD : Int, BB_WC : Int, BB_D : Int) extends BlackBox {
	val io = new  Bundle {
		val a = Bits(INPUT, width=BB_WA)
		val c = Bits(INPUT, width=BB_N*BB_WC)
		val d = Bits(INPUT, width=BB_N*BB_WD)
		val r = Bits(OUTPUT, width=BB_WA)
		val rst = Bool(INPUT)
		a.setName("a")
		c.setName("c")
		d.setName("d")
		r.setName("r")
        rst.setName("rst")
	}
	setVerilogParameters( new VerilogParameters {
		val WA 	  :Int = BB_WA
		val N 	  :Int = BB_N
		val WD    :Int = BB_WD
		val WC    :Int = BB_WC
		val DEPTH :Int = BB_D
	})
	//addResetPin(io.rst)
	addClock(Driver.implicitClock)

	//Behavioral model
	if (BB_WD == 1 && BB_WC == 1) {
		io.r := PopCount(io.c & io.d)
	}
}