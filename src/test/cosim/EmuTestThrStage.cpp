
// Author:  Davide Conficconi
// Date: 13/09/2018
// Revision: 0

#include <iostream>
#include "platform.h"
#include "EmuTestThrStage.hpp"
//Define according to the parameters of the Emulation flow
#define ROWS 2
#define COLS 3
#define THS 1


 // Cosim test for basic ThrStage behavior

 using namespace std;

 WrapperRegDriver * p;
 EmuTestThrStage * dut;

 //function prototypes
 void BramSingleTransfer(uint64_t *mat, int addr, bool activations, int sel_r, int sel_c);
 void BramTransferMat(uint64_t (&mat)[ROWS][COLS], int StartAddr);
 void testThr(uint8_t actOffset, uint8_t thrOffset, bool writeEn, uint8_t writeAddr);
 int32_t testres(uint8_t r, uint8_t c);
 uint8_t quantizeNumber(uint64_t * thrs, uint64_t number, int th_number);

 void BramSingleTransfer(uint64_t *mat ,int addr, bool activations, int sel_r, int sel_c){
  cout << "Single Transfer to BRAM of " << *mat; 
  cout << " at address "<< addr << " in act(1)/thr(0)"<< activations;
  cout << " Row "<< sel_r << " Cols " << sel_c << endl;
  if (activations)
  {
    dut->set_inMemory_act_sel_r(sel_r);
    dut->set_inMemory_act_sel_c(sel_c);
    dut->set_inMemory_act_addr(addr);
    dut->set_inMemory_act_data(*mat);
    dut->set_inMemory_act_write(1);
    dut->set_inMemory_act_write(0);

  }else{
    dut->set_inMemory_thr_sel_r(sel_r);
    dut->set_inMemory_thr_sel_c(sel_c);
    dut->set_inMemory_thr_addr(addr);
    dut->set_inMemory_thr_data(*mat);
    dut->set_inMemory_thr_write(1);
    dut->set_inMemory_thr_write(0);
  }
 }

 void BramTransferMat(uint64_t (&mat)[ROWS][COLS], int StartAddr){
  cout << "Activation Bram Transfer" << endl; 
  for(int i = 0; i<ROWS; i++)
    for(int j= 0; j<COLS; j++){
      dut->set_inMemory_act_sel_r(i);//
     dut->set_inMemory_act_addr(StartAddr); //
     dut->set_inMemory_act_sel_c(j);//
     dut->set_inMemory_act_data(mat[i][j]);//
     cout <<"Element "<< mat[i][j] << endl;
     dut->set_inMemory_act_write(1); //
     dut->set_inMemory_act_write(0); //
 }
}

//Starts the stage and the needed sw-configurable parameters
 void testThr(uint8_t actOffset, uint8_t thrOffset, bool writeEn, uint8_t writeAddr){
  cout << "Setting the controller and start the computation" << endl;
  dut->set_ctrl_thrOffset(thrOffset);
  dut->set_ctrl_actOffset(actOffset);
  dut->set_ctrl_writeAddr(writeAddr);
  dut->set_ctrl_writeEn(writeEn);
  dut->set_start(1);
  int count = 0;
  while ( dut->get_done() !=1 ){
   count++;
 }
  dut->set_start(0);
 }

 uint8_t quantizeNumber(uint64_t * thrs, uint64_t number, int th_number){
  uint8_t result = 0;
  cout << "Golden number" << endl;
  for (int i = 0; i < th_number; i++)
  {
    cout << number << " vs " << *thrs << endl;
    if (number > *thrs )
    {
      result += 1;
    }
  }
  return result;
 }

 int32_t testres(uint8_t r, uint8_t c){
   int32_t result;
   // read result memory at address 0 of the bram (r, c)
   dut->set_resmem_addr_r(r);
   dut->set_resmem_addr_c(c);
   dut->set_resmem_addr_e(0);
   result = dut->get_resmem_data() ;
   return result;
 }

 int main()
 {
   bool t_okay = false;
   try {
     p = initPlatform();
     dut = new EmuTestThrStage(p);  
    uint64_t a [ROWS][COLS] = {{0x0000000000000001,0x0000000000000002,0x0000000000000003},
                              {0x0000000000000004,0x0000000000000005,0x0000000000000006}};
    uint64_t th [ROWS][THS] = {{1},{0}};
    uint64_t gold [ROWS][COLS] = {{0x0,0x1,0x1},
                        {0x1,0x1,0x1}};
    uint64_t hw_res [ROWS][COLS];
    bool result_comparison [ROWS][COLS];

    BramTransferMat(a,0);
    for(int i = 0; i < ROWS; i++){
      for (int j = 0; j < THS; j++)
      {
        BramSingleTransfer(&(th[i][j]),0,false,i,j);
        
      }
    }

     // Actual Thresholding
   	cout << "Starting Thresholding" <<endl;
     testThr(0,0,true,0);
    for(int i = 0; i < ROWS; i++)
      for (int j = 0; j < COLS; j++){
        hw_res[i][j] = testres(i,j);
        cout << "HW Values :" << hw_res[i][j] << endl; 
      }
     cout << "Verification phase" << endl;
    for(int i = 0; i < ROWS; i++)
      for (int j = 0; j < COLS; j++){
        result_comparison[i][j] =  hw_res[i][j] == gold[i][j];
        cout << "Comparison :" << hw_res[i][j] << " vs "<< gold[i][j] << endl;
        t_okay = t_okay && result_comparison[i][j]; 
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
