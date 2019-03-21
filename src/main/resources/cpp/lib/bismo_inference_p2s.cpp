#include "bismo_inference_internal.hpp"

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
  // the p2s accelerator requires an aligned number of columns
  size_t ncols_a = gemmbitserial::alignTo(ncols, P2S_ALIGN);
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
  // clean the p2s buffer if desired
  if(zeropad) {
    // hand in a "cleanly padded" buffer to p2s
    uint8_t * in_clean = new uint8_t[nrows * ncols_a];
    memset(in_clean, 0, nrows * ncols_a * sizeof(uint8_t));
    platform->copyBufferHostToAccel((void *)in_clean, (void *)accel_p2s_bitpar_buffer, nbytes_aligned);
    delete [] in_clean;
  }
  // aligned copy the bit-parallel matrix into the accelerator
  for(size_t r = 0; r < nrows; r++) {
    platform->copyBufferHostToAccel(
      (void *)&host_buf_src[r * nbytes_row],
      (void *)(accel_p2s_bitpar_buffer + (r * nbytes_aligned_row)),
      nbytes_aligned_row);
  }
  // setup and call the p2s hardware accelerator
  acc->setup_p2s((void *)accel_p2s_bitpar_buffer, nbytes_bitser, (void *) accel_buf_dst, nrows, ncols_a, nbits);
  uint32_t cycles = acc->p2s_exec_and_wait();
  instrumentationData["run_p2s"] = (float) cycles;
}
}
