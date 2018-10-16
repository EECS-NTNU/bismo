// Author:  Davide Conficconi
// Date: 16/10/2018
// Revision: 0

/* Hardware debugger for chisel classes to print at runt-time*/

// Debug Levels: 0 info, 1 debug, 2 moderate, 3 failure 

package bismo

import Chisel._

//If user wants to declare its own debugger
class DebuggerClass (lvl:Int) extends Module{
  val io = new Bundle{
    val m = UInt(INPUT,32)

  }
  val debugLevel = lvl

  def log(msg: String, level: Int): Unit ={
    if(level >= debugLevel){
      printf(msg)
    }
  }
}

//Static default debugger
object Debugger {
  val debugger = Module(new DebuggerClass(2))

  def log(msg: String, level: Int): Unit ={
    if(level >= debugger.debugLevel){
      printf(msg)
    }
  }

}
