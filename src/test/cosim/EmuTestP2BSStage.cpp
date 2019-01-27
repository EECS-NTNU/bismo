
// Author:  Davide Conficconi
// Date: 27/09/2018
// Revision: 0

#include <iostream>
#include "platform.h"
#include "EmuTestP2BSStage.hpp"
//Define according to the parameters of the Emulation flow
#define ROWS 2
#define COLS 3
#define BW 4


 // Cosim test for basic ThrStage behavior

 using namespace std;

 WrapperRegDriver * p;
 EmuTestP2BSStage * dut;

 //function prototypes
 void BramTransferMat(uint64_t (&mat)[ROWS][COLS], int StartAddr);
 void testP2BS(uint8_t actOffset, uint8_t thrOffset, bool writeEn, uint8_t writeAddr);
 int32_t testres(uint8_t r, uint8_t addr);


 void BramTransferMat(uint64_t (&mat)[ROWS][COLS], int StartAddr){
  cout << "Activation Bram Transfer" << endl;
  for(int i = 0; i<ROWS; i++)
    for(int j= 0; j<COLS; j++){
      dut->set_inMemory_thr_sel_r(i);//
     dut->set_inMemory_thr_addr(StartAddr); //
     dut->set_inMemory_thr_sel_c(j);//
     dut->set_inMemory_thr_data(mat[i][j]);//
     cout <<"Element "<< mat[i][j] << endl;
     dut->set_inMemory_thr_write(1); //
     dut->set_inMemory_thr_write(0); //
 }
}

//Starts the stage and the needed sw-configurable parameters
 void testP2BS(uint8_t resOffset, uint8_t thrOffset, bool writeEn, uint8_t writeAddr, uint64_t bitcount){
  cout << "Setting the controller and start the computation" << endl;
  dut->set_ctrl_thrOffset(thrOffset);
  dut->set_ctrl_resOffset(resOffset);
  dut->set_ctrl_writeAddr(writeAddr);
  dut->set_ctrl_writeEn(writeEn);
  dut->set_ctrl_count_bits_shifting(bitcount);
  dut->set_start(1);
  int count = 0;
  while ( dut->get_done() !=1 ){
   count++;
 }
  dut->set_start(0);
 }


 int32_t testres(uint8_t r, uint8_t addr){
   int32_t result;
   // read result memory at address addr of the bram (r)
   dut->set_resmem_addr_r(r);
   dut->set_resmem_addr_e(addr);
   result = dut->get_resmem_data() ;
   return result;
 }

 int main()
 {
   bool t_okay = true;
   try {
     p = initPlatform();
     dut = new EmuTestP2BSStage(p);
    uint64_t a [ROWS][COLS] = {{0x0000000000000001,0x0000000000000002,0x0000000000000003},
                              {0x0000000000000004,0x0000000000000005,0x0000000000000006}};
    uint64_t gold [BW][ROWS] = {{0x5,0x2},{0x6,0x4},{0x0,0x7}};
    uint64_t hw_res [BW][ROWS];
    bool result_comparison [BW][ROWS];

    BramTransferMat(a,0);

     // Actual Thresholding
   	cout << "Starting Parallel2BitSerial" <<endl;
    testP2BS(0,0,true,0,BW);
    for(int j = 0; j < BW; j++)
      for(int i = 0; i < ROWS; i++){
        hw_res[j][i] = testres(i,j);
         cout << "HW Values :" << hw_res[j][i] << endl;
      }
    cout << "Verification phase" << endl;
    for(int j = 0; j < BW; j++)
      for(int i = 0; i < ROWS; i++){
        result_comparison[j][i] =  hw_res[j][i] == gold[j][i];
        cout << "Comparison :" << hw_res[j][i] << " vs "<< gold[j][i] << endl;
        t_okay = t_okay && result_comparison[j][i];
      }
     if(t_okay) {
       cout << "Test passed";
     } else {
       cout << "Test failed";
     }
     cout << endl;
     delete dut;
     deinitPlatform(p);
   } catch(const char * e) {
     cout << "Exception: " << e << endl;
   }
   return 0;//t_okay ? 0 : -1;
 }
