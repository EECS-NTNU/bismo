// Author:  Davide Conficconi
// Date: 19/10/2018
// Revision: 0


#include <cassert>
#include <iostream>
#include <vector>
#include <chrono>
#include <fstream>
#include "platform.h"
#include "EmuTestP2SAccel.hpp"
#include "gemmbitserial/gemmbitserial.hpp"
#include "gemmbitserial/test/testhelpers.hpp"

using namespace std;
using namespace gemmbitserial;

#define P2S_ALIGN 64
// TODO Yaman: consider exposing from generated header file
#define MAX_ACCEL_BITWIDTH 8
#define PLATFORM "PYNQZ1" //"PYNQU96"
#define AVG_BENCHMARK 1 
#define REPETITIONS 30

WrapperRegDriver * p;
EmuTestP2SAccel * dut;

uint32_t exec_and_wait() {
  dut->set_enable(1);
  dut->set_ackqueue_ready(false);
  while(dut->get_ackqueue_valid() != 1);
  uint32_t ret = dut->get_ackqueue_bits();
  // pulse ackqueue.ready to consume ack token
  dut->set_ackqueue_ready(true);
  dut->set_ackqueue_ready(false);
  dut->set_enable(0);
  return ret;
}

void setup_p2s(
  void * accel_buf_src,
  uint32_t nbytes,
  void * accel_buf_dst,
  uint32_t rows,
  uint32_t cols,
  uint32_t bit_width
) {
  // ensure #cols is divisible by P2S_ALIGN
  assert(cols % P2S_ALIGN == 0);
  size_t col_groups = cols / P2S_ALIGN;
  dut->set_cmdqueue_valid(false);
  dut->set_cmdqueue_bits_dramBaseAddrSrc((AccelDblReg) accel_buf_src);
  // cout << "[SW] DRAM Base Addr Src" << accel_buf << endl;
  dut->set_cmdqueue_bits_dramBaseAddrDst((AccelDblReg) accel_buf_dst );
  dut->set_cmdqueue_bits_matrixRows(rows);
  dut->set_cmdqueue_bits_matrixColsGroup(col_groups);
  dut->set_cmdqueue_bits_actualPrecision(bit_width);
  dut->set_cmdqueue_bits_waitCompleteBytes(nbytes);
  // wait until cmdqueue is available
  while(dut->get_cmdqueue_ready() != 1);
  // pulse cmdqueue.valid
  dut->set_cmdqueue_valid(true);
  dut->set_cmdqueue_valid(false);
}

int main(int argc, char * argv [])
{
  double frequency;
  if(argc == 1){
    frequency = 100;
  }else{
    frequency = atoi(argv[1]);
  }
  cout << "Target frequency: " << frequency << endl;
  auto start_main = std::chrono::system_clock::now();
  bool all_OK = true;
  p = initPlatform();
  dut = new EmuTestP2SAccel(p);
  vector<size_t> test_colgroups {1};//, 2, 3};//, 8, 9, 10, 18, 20};
  vector<size_t> test_rows {1};//, 2, 3};//, 4, 5, 6, 7, 8, 9, 10, 11, 12, 20};
  vector<size_t> test_bits {1};//, 2, 3, 4, 5, 6, 7};
  // required to be able to compute golden vectors with gemmbitserial
  assert(P2S_ALIGN % 64 == 0);
  // TODO check that max bit precision is set to 8 for the HW,
  // test code assumes this (the bit-parallel elem type is uint8_t)

  //Output results logging
  auto working = std::chrono::system_clock::now();
  std::time_t working_time = std::chrono::system_clock::to_time_t(working);
  ofstream logger;
  logger.open ("benchmark.log");
  logger << "Benchmarking results of P2S at" << std::ctime(&working_time) << "\n";
  logger << "Bits\tRows\tColumns\t" << "PS[ns]\t"; 

  if(AVG_BENCHMARK) {
    logger << "PS-AVG-" << REPETITIONS << "[ns]\t"; 
  }

  logger << "PL[cc]\tPL[ns]\tPLATFORM\n";

  for(auto & colgroups : test_colgroups) {
    for(auto & rows : test_rows) {
      for(auto & bits : test_bits) {
        size_t cols = colgroups * P2S_ALIGN;
        BitSerialMatrix bsm = BitSerialMatrix::alloc(
          bits, rows, cols, false, 1, P2S_ALIGN
        );
        logger << bits << "\t" << rows << "\t" << cols << "\t";
        cout << "Now running test: ";
        bsm.printSummary();
        // requested col count is already divisible by alignment
        // so #aligned cols should be the same as #cols
        assert(bsm.ncols_a == cols);
        uint8_t * bpm = new uint8_t[rows * cols];
        // generate a random matrix of desired shape and bits
        generateRandomVector(bits, rows * cols, bpm);
        // convert to bit-serial format in software


        //BENCHMARKING
        auto start = std::chrono::system_clock::now();
        bsm.importRegular(bpm);
        auto end = std::chrono::system_clock::now();
        double nseconds = std::chrono::duration_cast<std::chrono::nanoseconds>(end - start).count();
        std::cout << "Software elapsed time: " << nseconds << "ns\n";
        logger << nseconds << "\t";

        if(AVG_BENCHMARK){
          auto start_rep = std::chrono::system_clock::now();
          for (int i = 0; i < REPETITIONS; i++)
          {
            bsm.importRegular(bpm);
          }
          auto end_rep = std::chrono::system_clock::now();
          double nseconds_rep = std::chrono::duration_cast<std::chrono::nanoseconds>(end_rep - start_rep).count() / (double) REPETITIONS;
          std::cout << "Software average elapsed time: " << nseconds_rep << "ns for" << REPETITIONS << "times \n";
          logger << nseconds_rep << "\t";
        }

        // now do the same in hardware
        // compute buffer sizes
        size_t nbytes_bitpar = rows * cols * sizeof(uint8_t);
        size_t nbytes_bitser = (bits * rows * cols) / 8;
        // we'll memcmp the SW and HW results, so buffers must be exactly the
        // same size
        assert(sizeof(uint64_t) * bsm.wordsPerBitplane() * bits == nbytes_bitser);
        void * hw_src = p->allocAccelBuffer(nbytes_bitpar);
        void * hw_dst = p->allocAccelBuffer(nbytes_bitser);
        // set up inputs to accelerator and launch
        p->copyBufferHostToAccel(bpm, hw_src, nbytes_bitpar);
        setup_p2s(hw_src, nbytes_bitser, hw_dst, rows, cols, bits);
        uint32_t cycles = exec_and_wait();
        cout << "Took " << cycles << " clock cycles" << endl;

        cout << "Read respons ch tot cc: " << dut->get_momRd_totalCycles() << endl;
        cout << "Read respons ch active cc: " << dut->get_momRd_activeCycles() << endl;
        cout << "Read respons ch noVbutR cc: " << dut->get_momRd_noValidButReady() << endl;
        cout << "Read respons ch noRbutV cc: " << dut->get_momRd_noReadyButValid() << endl;
        cout << endl;
        cout << "Read request ch tot cc: " << dut->get_momRdRq_totalCycles() << endl;
        cout << "Read request ch active cc: " << dut->get_momRdRq_activeCycles() << endl;
        cout << "Read request ch noVbutR cc: " << dut->get_momRdRq_noValidButReady() << endl;
        cout << "Read request ch noRbutV cc: " << dut->get_momRdRq_noReadyButValid() << endl;
        cout << endl;
        cout << "Write respons ch tot cc: " << dut->get_momWr_totalCycles() << endl;
        cout << "Write respons ch active cc: " << dut->get_momWr_activeCycles() << endl;
        cout << "Write respons ch noVbutR cc: " << dut->get_momWr_noValidButReady() << endl;
        cout << "Write respons ch noRbutV cc: " << dut->get_momWr_noReadyButValid() << endl;


        logger << cycles << "\t" << cycles/frequency*1000 << "\t" << PLATFORM << "\n";
        // copy bit-serial matrix generated by hardware to host and compare
        uint8_t * hw_res = new uint8_t[nbytes_bitser];
        p->copyBufferAccelToHost(hw_dst, hw_res, nbytes_bitser);
        int memcmpres = memcmp(hw_res, bsm.data, nbytes_bitser);
        cout << "memcmp result " << memcmpres << endl;
        if(memcmpres != 0) {
          cout << "expected: " << endl;
          bsm.printHex();
          cout << "found: " << endl;
          memcpy(bsm.data, hw_res, nbytes_bitser);
          bsm.printHex();
        }
        all_OK &= (memcmpres == 0);
        delete [] bpm;
        delete [] hw_res;
        p->deallocAccelBuffer(hw_src);
        p->deallocAccelBuffer(hw_dst);
      }
    }
  }

  delete dut;
  deinitPlatform(p);
  logger.close();

  if(all_OK) {
    cout << "All tests passed" << endl;
  } else {
    cout << "Some tests failed" << endl;
  }

  auto end_main = std::chrono::system_clock::now();
  double seconds_main = std::chrono::duration<double>(end_main - start_main).count();
  std::time_t start_main_time = std::chrono::system_clock::to_time_t(start_main);
  std::time_t end_main_time = std::chrono::system_clock::to_time_t(end_main);
  std::cout << "Exect started: " << std::ctime(&start_main_time) << "ends  at "<< std::ctime(&end_main_time);
  std::cout << "\nSoftware elapsed time: " << seconds_main << "s\n";
  return all_OK ? 0 : -1;
}
