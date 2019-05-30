#include "bismo_rt_matmul.hpp"

namespace bismo_inference {

LayerHandle initMatMulLayer(MatMulLayerDescriptor & dsc) {
  Matrix<uint8_t> * lhs = new Matrix<uint8_t>(
    dsc.M, dsc.K, dsc.wbits, dsc.wsigned, false, matTypeLHS
  );
  Matrix<uint8_t> * rhs = new Matrix<uint8_t>(
    dsc.K, dsc.N, dsc.ibits, dsc.isigned, true, matTypeRHS
  );
  Matrix<int32_t> * res = new Matrix<int32_t>(
    dsc.M, dsc.N, 32, true, true, matTypeRes
  );
  MatrixMultiply * mm = new MatrixMultiply(lhs, rhs, res);

  return (LayerHandle) mm;
}

void execMatMulLayer(LayerHandle id) {
  MatrixMultiply * mm = (MatrixMultiply *) id;
  mm->exec();
}

uint8_t * getLayerLHSBuffer(LayerHandle id) {
  MatrixMultiply * mm = (MatrixMultiply *) id;
  return mm->m_lhs->hostbuf();
}

uint8_t * getLayerRHSBuffer(LayerHandle id) {
  MatrixMultiply * mm = (MatrixMultiply *) id;
  return mm->m_rhs->hostbuf();
}

int32_t * getLayerResBuffer(LayerHandle id) {
  MatrixMultiply * mm = (MatrixMultiply *) id;
  return mm->m_res->hostbuf();
}

void syncLayerLHSBuffer(LayerHandle id) {
  MatrixMultiply * mm = (MatrixMultiply *) id;
  mm->m_lhs->host2accel();
}

void syncLayerRHSBuffer(LayerHandle id) {
  MatrixMultiply * mm = (MatrixMultiply *) id;
  mm->m_rhs->host2accel();
}

void syncLayerResBuffer(LayerHandle id) {
  MatrixMultiply * mm = (MatrixMultiply *) id;
  mm->m_res->accel2host();
}

void deinitLayer(LayerHandle id) {
  MatrixMultiply * mm = (MatrixMultiply *) id;
  delete mm->m_lhs;
  delete mm->m_rhs;
  delete mm->m_res;
  delete mm;
}

MatrixMultiply::MatrixMultiply(
  Matrix<uint8_t> * lhs, Matrix<uint8_t> * rhs, Matrix<int32_t> * res
) {
  m_lhs = lhs;
  m_rhs = rhs;
  m_res = res;
  // ensure sizes are compatible
  if(m_lhs->inner() != m_rhs->inner()) {
    throw "LHS/RHS dimensions are incompatible";
  }
  if((m_lhs->outer() != m_res->inner()) || (m_rhs->outer() != m_res->outer())) {
    throw "Result dimensions are incompatible with input(s)";
  }
  const size_t tiles_m = m_lhs->outer_a() / cfg.dpaDimLHS;
  const size_t tiles_k = m_lhs->inner_a() / cfg.dpaDimCommon;
  const size_t tiles_n = m_rhs->outer_a() / cfg.dpaDimRHS;
  const size_t lhs_stripe_nbytes = m_lhs->bitserial_nbytes() / tiles_m;
  const size_t rhs_stripe_nbytes = m_rhs->bitserial_nbytes() / tiles_n;
  // must have room for at least one stripe per bit position, as this is the
  // granularity we at which we do RHS tiling
  const bool rhs_tile_fits_in_ocm = (acc->get_rhs_total_BRAM_bytes()) >= FETCHEXEC_TOKENS*rhs_stripe_nbytes;
  const bool rhs_tile_is_one_fetchblock = (rhs_stripe_nbytes <= FETCH_BLOCK_MAX);
  if(!rhs_tile_is_one_fetchblock || !rhs_tile_fits_in_ocm) {
    throw "RHS is too large and not currently supported in runtime library.";
  }
  const bool lhs_tile_fits_in_ocm = (acc->get_lhs_total_BRAM_bytes()) >= FETCHEXEC_TOKENS*lhs_stripe_nbytes;
  const bool lhs_tile_is_one_fetchblock = lhs_stripe_nbytes <= FETCH_BLOCK_MAX;
  if(!lhs_tile_is_one_fetchblock || !lhs_tile_fits_in_ocm) {
    throw "LHS is too large and not currently supported in runtime library.";
  }
  // create and fill in the descriptor
  m_igen_dsc.tiles_m = tiles_m;
  m_igen_dsc.tiles_k = tiles_k;
  m_igen_dsc.tiles_n = tiles_n;
  m_igen_dsc.bits_l = m_lhs->bits();
  m_igen_dsc.bits_r = m_rhs->bits();
  m_igen_dsc.signed_l = m_lhs->is_signed();
  m_igen_dsc.signed_r = m_rhs->is_signed();
  m_igen_dsc.base_l = 0;
  m_igen_dsc.base_r = 0;
  m_igen_dsc.base_res = 0;
  m_igen_dsc.nbufs_fetch_exec_log2 = FETCHEXEC_TOKENS_LOG2;
  m_igen_dsc.dram_lhs = m_lhs->bitserial_accelbuf();
  m_igen_dsc.dram_rhs = m_rhs->bitserial_accelbuf();
  m_igen_dsc.dram_res = m_res->accelbuf();
};

// note: deallocation of MatrixMultiply does NOT free the LHS/RHS/res matrices
MatrixMultiply::~MatrixMultiply() {};

void MatrixMultiply::exec() {
  // TODO instrumentation
  acc->set_stage_enables(0, 0, 0);
  acc->useDescriptors();
  // feed the instrgen descriptor
  acc->pushSingleMMDescriptor(m_igen_dsc);
  // HACK: make sure at least one op has appeared before checking for completion
  // proper way to fix this is to singal completion from accel explicitly
  while(acc->res_opcount() == 0) {};
  // enable all stages
  acc->set_stage_enables(1, 1, 1);
  // wait until all writes are completed
  while(acc->res_opcount() != 0) {};
};

}
