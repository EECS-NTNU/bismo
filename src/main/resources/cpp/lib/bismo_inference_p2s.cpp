#include "bismo_inference_internal.hpp"

namespace bismo_inference {
// bit-parallel to bit-serial conversion
void p2s(const uint8_t * host_buf, uint32_t accel_buf, gemmbitserial::BitSerialMatrix & mat) {
  size_t nbytes = mat.wordsPerBitplane() * mat.nbits * sizeof(PackedBitGroupType);
  auto start_time = std::chrono::high_resolution_clock::now();
  auto end_time = std::chrono::high_resolution_clock::now();
#ifdef FORCE_SW_P2S
  // force software p2s, useful to rule out hw p2s bugs
  start_time = std::chrono::high_resolution_clock::now();
  mat.importRegular((uint8_t *)host_buf);
  platform->copyBufferHostToAccel(mat.data, (void *) accel_buf, nbytes);
  end_time = std::chrono::high_resolution_clock::now();
  auto rhs2bs_duration_time = std::chrono::duration_cast<std::chrono::microseconds>(end_time-start_time).count();
  BISMORT_DEBUG("[execMatMulLayer] software bit-serialization+copy time: " << rhs2bs_duration_time << " us" );
#else
  // use hardware p2s
  // Create a tmp in buff for parallel activations
  // TODO this is needed only once per network (allocate for largest activation),
  // infrastructure functions (alloc etc) should not be called during inference
  size_t nbytes_bitpar = mat.nrows_a * mat.ncols_a * sizeof(uint8_t);
  void * inbuf_p2s = platform->allocAccelBuffer(nbytes_bitpar);
#ifdef P2S_CLEAR_IN_BUF
  // hand in a "cleanly padded" buffer to p2s
  uint8_t * in_formatted = new uint8_t[mat.nrows_a * mat.ncols_a];
  memset(in_formatted, 0, mat.nrows_a * mat.ncols_a);
  memcpy(in_formatted, in, rhs.nrows * rhs.ncols);
  platform->copyBufferHostToAccel((void *)in_formatted, inbuf_p2s, nbytes_bitpar);
  delete [] in_formatted;
#else
  platform->copyBufferHostToAccel((void *)host_buf, inbuf_p2s, nbytes_bitpar);
#endif
  //use bismo input buff as out buff for p2s
  acc->setup_p2s(inbuf_p2s, nbytes, (void *) accel_buf, mat.nrows_a, mat.ncols_a, mat.nbits);
  uint32_t cycles = acc->p2s_exec_and_wait();
  BISMORT_DEBUG("[execMatMulLayer] Serialize rhs took " << cycles << " cycles");

#ifdef BISMORT_P2S_VERIFY_AGAINST_CPU
  // compare against sw version
  rhs.importRegular((uint8_t *) host_buf);
  uint8_t * rhs_hw_serialized = new uint8_t[nbytes];
  platform->copyBufferAccelToHost((void *)accel_buf, rhs_hw_serialized, nbytes);
  int memcmpres = memcmp(rhs_hw_serialized, mat.data, nbytes);
  if(memcmpres != 0){
  // copy buffer from host
#ifdef P2S_CLEAR_IN_BUF
  // that version cannot admit p2s errors
  assert(memcmpres==0);
#endif
  // platform->copyBufferHostToAccel(rhs.data, (void *)dsc.accel_buf_in, dsc.nbytes_buf_in);
  BISMORT_DEBUG("[execMatMulLayer] expected: ");
#ifdef DEBUG
  rhs.printHex();
#endif
  BISMORT_DEBUG("[execMatMulLayer] found: ");
  memcpy(mat.data, rhs_hw_serialized, nbytes);
#ifdef DEBUG
  rhs.printHex();
#endif
  } else {
    BISMORT_DEBUG("[execMatMulLayer] P2S hw execution fine");
  }
#endif
#endif
}
}
