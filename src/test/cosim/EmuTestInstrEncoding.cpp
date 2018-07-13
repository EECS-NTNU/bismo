// Copyright (c) 2018 Xilinx
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

#include <iostream>
#include "platform.h"
#include "EmuTestInstrEncoding.hpp"
#include <bitset>
#include "BISMOInstruction.hpp"
#include <string.h>

// Cosim test for checking instruction encoding

using namespace std;

WrapperRegDriver * p;
EmuTestInstrEncoding * t;

int main()
{
  bool t_okay = true;
  try {
    p = initPlatform();
    t = new EmuTestInstrEncoding(p);


    BISMOSyncInstruction sw_ins, hw_ins;
    /*for(int i = 0; i < 4; i++) {
      cout << "raw " << i << " " << bitset<32>(sw_ins.raw[i]) << endl;
    }*/
    sw_ins.isRunCfg = 1;
    sw_ins.targetStage = 2;
    sw_ins.isSendToken = 0;
    sw_ins.chanID = 3;
    /*for(int i = 0; i < 4; i++) {
      cout << "raw " << i << " " << bitset<32>(sw_ins.raw[i]) << endl;
    }*/

    // BitFieldMember uses big-endian packing but
    // Chisel expects little-endian, so swap storage order here
    t->set_raw_instr_in0(sw_ins.raw[3]);
    t->set_raw_instr_in1(sw_ins.raw[2]);
    t->set_raw_instr_in2(sw_ins.raw[1]);
    t->set_raw_instr_in3(sw_ins.raw[0]);

    hw_ins.isRunCfg = t->get_sync_instr_out_isRunCfg();
    hw_ins.targetStage = t->get_sync_instr_out_targetStage();
    hw_ins.isSendToken = t->get_sync_instr_out_isSendToken();
    hw_ins.chanID = t->get_sync_instr_out_chanID();

    t_okay &= (memcmp(sw_ins.raw, hw_ins.raw, 16) == 0);

    if(t_okay) {
      cout << "All tests passed" << endl;
    } else {
      cout << "Some tests failed" << endl;
    }

    deinitPlatform(p);
  } catch(const char * e) {
    cout << "Exception: " << e << endl;
  }
  return t_okay ? 0 : -1;
}
