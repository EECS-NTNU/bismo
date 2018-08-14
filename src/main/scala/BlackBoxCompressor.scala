package bismo

import Chisel._


class BlackBoxCompressor(N : Int) extends Module {
	def outputbits = log2Up(N) + 1
	val io = new Bundle{
		val c = Bits(INPUT, width = N)
		val d = Bits(INPUT, width = N)
		val r = Bits(OUTPUT, width = outputbits)
	}

	val inst = Module(new mac(BB_WA = outputbits, BB_N = N, BB_WD = 1, BB_WC = 1, BB_HAVE_ACCU_INPUT = false)).io
	inst.a := UInt(0)
	inst <> io 
}

class mac (BB_WA : Int, BB_N: Int, BB_WD : Int, BB_WC : Int, BB_HAVE_ACCU_INPUT : Boolean) extends BlackBox {
	val io = new  Bundle {
		val a = Bits(INPUT, width=BB_WA)
		val c = Bits(INPUT, width=BB_N*BB_WC)
		val d = Bits(INPUT, width=BB_N*BB_WD)
		val r = Bits(OUTPUT, width=BB_WA)
	}
	setVerilogParameters( new VerilogParameters {
		val WA:Int = BB_WA
		val N:Int = BB_N
		val WD:Int = BB_WD
		val WC:Int = BB_WC
		val HAVE_ACCU_INPUT:Boolean = BB_HAVE_ACCU_INPUT
	})

	addClock(Driver.implicitClock)

	//Behavioral model
	if (BB_WD == 1 && BB_WC == 1) {
		io.r := PopCount(io.c & io.d)
	}
}