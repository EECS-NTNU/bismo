
// Author:  Davide Conficconi
// Date: 13/09/2018
// Revision: 0

 #include <iostream>
 #include "platform.h"
 #include "EmuTestThrStage.hpp"
#define N_ELEM 2


 // Cosim test for basic ThrStage behavior

 using namespace std;

 WrapperRegDriver * p;
 EmuTestThrStage * dut;

 //function prototypes
 void BramTransferthr(uint64_t *lmat, int StartAddr, int steps, int sel);
 void BramTransferact(uint64_t  *rmat, int StartAddr, int steps, int sel);
 void testmul(uint8_t tilenum, uint8_t shiftAmt, uint8_t l_offset, uint8_t r_offset, uint8_t neg, bool acc_clear);
 void testThr();
 void testres();

 void BramTransferthr(uint64_t *lmat, int StartAddr, int steps, int sel){
   // initialize the thr matrix memory
   cout << "starting thr"  << endl;
   dut->set_inMemory_thr_sel(sel); //
   for (int i = StartAddr ; i < StartAddr+steps ; i++){
     dut->set_inMemory_thr_addr(i); //
     dut->set_inMemory_thr_data(*lmat);
     cout <<"Element "<<i<<"  "<< *lmat << endl;
     dut->set_inMemory_thr_write(1); // select the
     dut->set_inMemory_thr_write(0); ///
     lmat++;
   }
 }

 /*
 This is for the right hand side
 */
 void BramTransferact(uint64_t *rmat, int StartAddr, int steps, int sel){
   cout << "starting act"  << endl;
   dut->set_inMemory_act_sel(sel); //
   for (int i = StartAddr ; i < StartAddr+steps ; i++){
     dut->set_inMemory_act_addr(i); //
     dut->set_inMemory_act_data(*rmat);
     cout <<"Element "<<i<<"  "<< *rmat << endl;
     dut->set_inMemory_act_write(1); //
     dut->set_inMemory_act_write(0); //
     rmat++;
   }
 }



 void testmul(
   uint8_t tilenum, uint8_t shiftAmt, uint8_t l_offset, uint8_t r_offset,
   uint8_t neg, bool acc_clear, uint8_t writeAddr, bool do_write
 ){
 	//dut->set_ctrl_negate(neg);
 	//dut->set_ctrl_shiftAmount(shiftAmt);
 	//dut->set_ctrl_numTiles(tilenum);
 	dut->set_ctrl_thrOffset(l_offset);
 	dut->set_ctrl_actOffset(r_offset);
   dut->set_ctrl_writeAddr(writeAddr);
   dut->set_ctrl_writeEn(do_write ? 1 : 0);
 	//dut->set_ctrl_clear_before_first_accumulation(acc_clear ? 1: 0);
   // launch the accelerator
 	dut->set_start(1);
 	while (dut->get_done() !=1){};
 	dut->set_start(0);
 	// read result memory at (0, 0)
 }

 int32_t testres(uint8_t r, uint8_t c){
   int32_t result;
   // read result memory at (0, 0)
   //dut->set_resmem_addr_c(c);
   dut->set_resmem_addr_r(r);
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
     // Matrix Arrays
     uint64_t i1[N_ELEM];
     uint64_t i2[N_ELEM];
     uint64_t i3[N_ELEM];
     uint64_t i4[N_ELEM];
     uint64_t *i1_p;
     uint64_t *i2_p;
     uint64_t *i3_p;
     uint64_t *i4_p;

     uint64_t resul[N_ELEM];

     uint64_t LAddr = 0;
     uint16_t RAddr = 0;
     uint64_t AddrStep = N_ELEM;
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
       i1[ic] = (uint64_t) 0x000000000000000f;
       i2[ic] = (uint64_t) 0x0000000000000009;
       i3[ic] = (uint64_t) 0x000000000000000f;
       i4[ic] = (uint64_t) 0x00000000000000af;
     }

   	// Pointers to the required addresses
   	i1_p = i1;
   	i2_p = i2;
   	i3_p = i3;
   	i4_p = i4;

   	// Begin Transfer
   	//  Initialize the BRAM with the variables
   	BramTransferact(i1_p,RAddr,AddrStep,0);
   	BramTransferact(i2_p,RAddr,AddrStep,1);
   	BramTransferthr(i3_p,LAddr,AddrStep,0);
   	BramTransferthr(i4_p,LAddr,AddrStep,1);

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
     cout << "Verification" << endl;
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
   return 0;//t_okay ? 0 : -1;
 }
