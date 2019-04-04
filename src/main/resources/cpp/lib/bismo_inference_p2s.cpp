#include "bismo_inference_internal.hpp"
#include "gemmbitserial/test/testhelpers.hpp"
#include <string>
namespace bismo_inference {
uint32_t accel_p2s_bitpar_buffer;

// hardware-accelerated 8-bit-parallel to bit-serial conversion
// the 8-bit is only the container datatype, can specify a smaller number
// of actual bits for the conversion
void p2s(
  const uint8_t * host_buf_src,   // input matrix buffer (source)
  uint32_t accel_buf_dst,         // output matrix buffer (destination)
  size_t nrows, size_t ncols,     // matrix size
  size_t nbits,                   // actual bits per element in source matrix
  bool issigned,                  // whether source matrix is signed
  bool zeropad                    // use zero instead of random padding
) {
#ifdef BISMORT_USE_SW_P2S
  // use gemmbitserial to do CPU import
  gemmbitserial::BitSerialMatrix mat = gemmbitserial::BitSerialMatrix::alloc(
    nbits, nrows, ncols, issigned, cfg.dpaDimRHS, cfg.dpaDimCommon
  );
  size_t nbytes_aligned = nbits * mat.wordsPerBitplane() * sizeof(PackedBitGroupType);
  memset(mat.data, 0, nbytes_aligned);
  mat.importRegular(host_buf_src);
  platform->copyBufferHostToAccel((void *)mat.data, (void *)accel_buf_dst, nbytes_aligned);
#else
  // TODO need row and col alignment to match accel reqs here
  // the p2s accelerator requires an aligned number of columns
  size_t ncols_a = gemmbitserial::alignTo(ncols, cfg.dpaDimCommon);
  size_t nbytes_aligned_row = ncols_a * sizeof(uint8_t);
  size_t nbytes_row = ncols * sizeof(uint8_t);
  size_t nbytes_aligned = nrows * ncols_a * sizeof(uint8_t);
  const size_t nbytes_bitser = (nrows * ncols_a * nbits) / 8;
  if(nbytes_aligned > BISMORT_P2S_BITPAR_BYTES) {
    throw "Insufficient p2s bit-parallel buffer size";
  }
  if(issigned) {
    throw "P2S accelerator does not yet support signed import";
  }
  // TODO optimization: allocate this only once
  uint8_t * in_clean = new uint8_t[nbytes_aligned];
  // clean the p2s buffer if desired
  if(zeropad) {
    // hand in a "cleanly padded" buffer to p2s
    memset(in_clean, 0, nbytes_aligned);
  }
  // aligned copy the bit-parallel matrix into the accelerator
  for(size_t r = 0; r < nrows; r++) {
    std::memcpy((void *)(in_clean + (r * nbytes_aligned_row)), (void *)&host_buf_src[r * nbytes_row], nbytes_row);
  }
  platform->copyBufferHostToAccel((void *)in_clean, (void *)accel_p2s_bitpar_buffer, nbytes_aligned);
  delete [] in_clean;
  // setup and call the p2s hardware accelerator
  acc->setup_p2s((void *)accel_p2s_bitpar_buffer, nbytes_bitser, (void *) accel_buf_dst, nrows, ncols_a, nbits);
  uint32_t cycles = acc->p2s_exec_and_wait();
  instrumentationData["run_p2s"] = (float) cycles;
#endif
}

bool selftest_p2s() {
  bool ret = true;
  vector<size_t> nbits_alts {1, 2, 3};
  vector<size_t> spatial_alts_r {1, 10, 20};
  vector<size_t> spatial_alts_c {100, 256, 512, 1000};
  for(auto & nbits: nbits_alts) {
    for(auto & nrows: spatial_alts_r) {
      for(auto & ncols: spatial_alts_c) {
        bool issigned = false;
        string test_name = "p2s_" + to_string(nrows) + "x" + to_string(ncols) + "_" + to_string(nbits) +"b_" + (issigned ? "s" : "u");
        cout << "Starting test:" << test_name << endl;
        uint8_t * mat_bp = new uint8_t[nrows * ncols];
        gemmbitserial::generateRandomVector(nbits, nrows*ncols, mat_bp);
        gemmbitserial::BitSerialMatrix mat_bs = gemmbitserial::BitSerialMatrix::alloc(
          nbits, nrows, ncols, issigned, cfg.dpaDimRHS, cfg.dpaDimCommon
        );
        TIMER_SAMPLE();
        mat_bs.importRegular(mat_bp);
        TIMER_SAMPLE();
        TIMER_REPORT("run_p2s_benchmark_sw");
        size_t nbytes_bitser = mat_bs.wordsPerBitplane() * nbits * sizeof(PackedBitGroupType);
        uint32_t accel_buf = (uint32_t)(uint64_t)platform->allocAccelBuffer(nbytes_bitser);
        // call p2s with forced zero-padding
        p2s(mat_bp, accel_buf, nrows, ncols, nbits, issigned, true);
        // copy result back to host
        uint8_t * accel_mat_bs = new uint8_t[nbytes_bitser];
        platform->copyBufferAccelToHost((void *)accel_buf, accel_mat_bs, nbytes_bitser);
        ret &= (memcmp(accel_mat_bs, mat_bs.data, nbytes_bitser) == 0);
        /*if(!ret) {
          for(size_t i = 0; i < nbytes_bitser; i++) {
            if(accel_mat_bs[i] != ((uint8_t*)mat_bs.data)[i]) {
              cout << i << "\t" <<  hex << (int)accel_mat_bs[i] << "\t" << (int)((uint8_t*)mat_bs.data)[i] << dec << endl;
            }
          }
        }*/
        cout << test_name << "\tok? = " << ret << "\tcycles = " << instrumentationData["run_p2s"];
        cout << " software importRegular us = " << instrumentationData["run_p2s_benchmark_sw"] << endl;
        platform->deallocAccelBuffer((void *)accel_buf);
        delete [] accel_mat_bs;
        delete [] mat_bp;
      }
    }
  }
  cout << "All tests passed? " << ret << endl;
  return ret;
}
}
