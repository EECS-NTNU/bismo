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

#include <iostream>
#include "platform.h"
#include "EmuTestExecStage.hpp"

// Cosim test for basic ExecStage behavior

using namespace std;

WrapperRegDriver * p;
EmuTestExecStage * t;

//function prototypes
void BramTransferLHS(uint64_t *lmat, int StartAddr, int steps, int sel);
void BramTransferRHS(uint64_t  *rmat, int StartAddr, int steps, int sel);
void testmul(uint8_t tilenum, uint8_t shiftAmt, uint8_t l_offset, uint8_t r_offset, uint8_t neg, bool acc_clear);
void testres();

void BramTransferLHS(uint64_t *lmat, int StartAddr, int steps, int sel){
  // initialize the LHS matrix memory
  cout << "starting lhs"  << endl;
  t->set_tilemem_lhs_sel(sel); //
  for (int i = StartAddr ; i < StartAddr+steps ; i++){
    t->set_tilemem_lhs_addr(i); //
    t->set_tilemem_lhs_data(*lmat);
    cout <<"Element "<<i<<"  "<< *lmat << endl;
    t->set_tilemem_lhs_write(1); // select the
    t->set_tilemem_lhs_write(0); ///
    lmat++;
  }
}

/*
This is for the right hand side
*/
void BramTransferRHS(uint64_t *rmat, int StartAddr, int steps, int sel){
  cout << "starting rhs"  << endl;
  t->set_tilemem_rhs_sel(sel); //
  for (int i = StartAddr ; i < StartAddr+steps ; i++){
    t->set_tilemem_rhs_addr(i); //
    t->set_tilemem_rhs_data(*rmat);
    cout <<"Element "<<i<<"  "<< *rmat << endl;
    t->set_tilemem_rhs_write(1); //
    t->set_tilemem_rhs_write(0); //
    rmat++;
  }
}



void testmul(
  uint8_t tilenum, uint8_t shiftAmt, uint8_t l_offset, uint8_t r_offset,
  uint8_t neg, bool acc_clear, uint8_t writeAddr, bool do_write
){
	t->set_csr_negate(neg);
	t->set_csr_shiftAmount(shiftAmt);
	t->set_csr_numTiles(tilenum);
	t->set_csr_lhsOffset(l_offset);
	t->set_csr_rhsOffset(r_offset);
  t->set_csr_writeAddr(writeAddr);
  t->set_csr_writeEn(do_write ? 1 : 0);
	t->set_csr_clear_before_first_accumulation(acc_clear ? 1: 0);
  // launch the accelerator
	t->set_start(1);
	while (t->get_done() !=1){};
	t->set_start(0);
	// read result memory at (0, 0)
}

int32_t testres(uint8_t r, uint8_t c){
  int32_t result;
  // read result memory at (0, 0)
  t->set_resmem_addr_c(c);
  t->set_resmem_addr_r(r);
  t->set_resmem_addr_e(0);
  result = t->get_resmem_data() ;
  return result;
}



int main()
{
  bool t_okay = false;
  try {
    p = initPlatform();
    t = new EmuTestExecStage(p);
    // Matrix Arrays
    uint64_t i1[10];
    uint64_t i2[10];
    uint64_t i3[10];
    uint64_t i4[10];
    uint64_t *i1_p;
    uint64_t *i2_p;
    uint64_t *i3_p;
    uint64_t *i4_p;

    uint64_t resul[10];

    uint64_t LAddr = 0;
    uint16_t RAddr = 0;
    uint64_t AddrStep = 10;
    uint8_t  tiles = 2;
    uint8_t  shift = 1;

    int32_t r00,r01,r10,r11;
    int32_t c00,c01,c10,c11;

    bool t00 = true;
    bool t01 = true;
    bool t10 = true;
    bool t11 = true;

    t_okay = true;
    // create a known test matrix
    // The matrix is 2.dpex * 2.dpez times, same for dpey
    for (int ic = 0; ic <AddrStep; ic ++ ){
      i1[ic] = (uint64_t) 0x00000000000000ff;
      i2[ic] = (uint64_t) 0x00000000000008f0;
      i3[ic] = (uint64_t) 0x00000000000000af;
      i4[ic] = (uint64_t) 0x0000000000000af0;
    }

  	// Pointers to the required addresses
  	i1_p = i1;
  	i2_p = i2;
  	i3_p = i3;
  	i4_p = i4;

  	// Begin Transfer
  	//  Initialize the BRAM with the variables
  	BramTransferRHS(i1_p,RAddr,AddrStep,0);
  	BramTransferRHS(i2_p,RAddr,AddrStep,1);
  	BramTransferLHS(i3_p,LAddr,AddrStep,0);
  	BramTransferLHS(i4_p,LAddr,AddrStep,1);

    // Actual Multiplication
  	cout << "Starting Multiply" <<endl;

    testmul(tiles,shift,0,0,0,true,0,true);
    r00 = testres(0,0);
    r01 = testres(0,1);
    r10 = testres(1,0);
    r11 = testres(1,1);

    cout << "Expected Values for this test" << endl;
    c00 = 6*tiles*(shift+1);
    c01 = 2*tiles*(shift+1);
    c10 = 4*tiles*(shift+1);
    c11 = 5*tiles*(shift+1);

    t00 = c00 == r00 ;
    t01 = c01 == r01 ;
    t10 = c10 == r10 ;
    t11 = c11 == r11 ;

    t_okay = t00 && t01 && t10 && t11;
    cout <<r00<<endl;
    cout <<r01<<endl;
    cout <<r10<<endl;
    cout <<r11<<endl;
    cout <<c00<<endl;
    cout <<c01<<endl;
    cout <<c10<<endl;
    cout <<c11<<endl;
    cout <<t00<<endl;
    cout <<t01<<endl;
    cout <<t10<<endl;
    cout <<t11<<endl;
    if(t_okay) {
      cout << "Test passed";
    } else {
      cout << "Test failed";
    }
    cout << endl;

    delete t;
    deinitPlatform(p);
  } catch(const char * e) {
    cout << "Exception: " << e << endl;
  }
  return t_okay ? 0 : -1;
}
