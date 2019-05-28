/*
#include "bismo_inference_internal.hpp"

namespace bismo_inference {

void verifyConv(
  gemmbitserial::ConvBitSerialContext ctx,
  const uint8_t * in,
  const int32_t * out
) {
  // NOTE: lhs and rhs are swapped, see note in initConvLayer
  gemmbitserial::BitSerialMatrix lhs = ctx.gemmctx.rhs;
  gemmbitserial::BitSerialMatrix rhs = ctx.gemmctx.lhs;
  uint64_t checksum_lhs = 0;
  uint64_t checksum_rhs = 0;
  uint32_t checksum_res = 0;
  for(size_t i = 0; i < lhs.nbits * lhs.wordsPerBitplane(); i++) checksum_lhs += lhs.data[i];
  for(size_t i = 0; i < rhs.nbits * rhs.wordsPerBitplane(); i++) checksum_rhs += rhs.data[i];
  for(size_t i = 0; i < lhs.nrows * rhs.nrows; i++) checksum_res += out[i];
  BISMORT_DEBUG("[execConvLayer] LHS checksum = " << hex << checksum_lhs << dec << endl);
  BISMORT_DEBUG("[execConvLayer] RHS checksum = " << hex << checksum_rhs << dec << endl);
  BISMORT_DEBUG("[execConvLayer] res checksum = " << hex << checksum_res << dec << endl);
  // import activations
  ctx.importActivations(in);
  // compute result with CPU and compare
  size_t actual_res_bytes = sizeof(AccumType) * lhs.nrows * rhs.nrows;
  gemmbitserial::gemmBitSerial(ctx.gemmctx);
  int ret = memcmp(ctx.gemmctx.res, out, actual_res_bytes);
  BISMORT_DEBUG("[execConvLayer] memcmp against golden = " << ret);
  //const size_t bs_w_bytes = ctx.gemmctx.rhs.nbits * ctx.gemmctx.rhs.wordsPerBitplane() * sizeof(PackedBitGroupType);
  //int cmp_wmat = memcmp(ctx.gemmctx.rhs.data, dsc_matmul.ctx.lhs.data, bs_w_bytes);
  //int cmp_amat = memcmp(ctx.gemmctx.lhs.data, dsc_matmul.ctx.rhs.data, lowered_bs_act_bytes);
  //BISMORT_DEBUG("[execConvLayer] W matrix memcmp = " << cmp_wmat);
  //BISMORT_DEBUG("[execConvLayer] A matrix memcmp = " << cmp_amat);
  if(ret != 0) {
    cout << "expected vs found" << endl;
    for(int i = 0; i < lhs.nrows * rhs.nrows; i++) {
      if(ctx.gemmctx.res[i] != out[i]) {
        cout << "pos " << i << ": " << ctx.gemmctx.res[i] << " " << out[i] << endl;
      }
    }
  }
}

}
*/
