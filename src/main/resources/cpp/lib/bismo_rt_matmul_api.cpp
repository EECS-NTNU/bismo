#include "bismo_rt_matmul.hpp"
#include <iostream>
#include "gemmbitserial/test/testhelpers.hpp"

namespace bismo_rt {
// note that the MatMul calls here simply implement wrappers around the
// MatrixMultiply class. this is to allow the BISMO RT to be compiled as a
// shared library, also removing the need for the template classes implemented
// as header files to be included with the rtlib

LayerHandle initMatMulLayer(MatMulLayerDescriptor & dsc) {
  Matrix<uint8_t> * lhs = new Matrix<uint8_t>(
    dsc.M, dsc.K, dsc.wbits, dsc.wsigned, false, matTypeLHS, "mat_lhs"
  );
  Matrix<uint8_t> * rhs = new Matrix<uint8_t>(
    dsc.K, dsc.N, dsc.ibits, dsc.isigned, true, matTypeRHS, "mat_rhs"
  );
  Matrix<int32_t> * res = new Matrix<int32_t>(
    dsc.M, dsc.N, 32, true, true, matTypeRes, "mat_res"
  );
#ifdef BISMORT_MATMUL_VERIFY_AGAINST_CPU
  bool allow_gemmbitserial = true;
#else
  bool allow_gemmbitserial = false;
#endif
  MatrixMultiply * mm = new MatrixMultiply(lhs, rhs, res, allow_gemmbitserial);

  return (LayerHandle) mm;
}

void execMatMulLayer(LayerHandle id) {
  MatrixMultiply * mm = (MatrixMultiply *) id;
  mm->exec();
  if(mm->has_cpu_ctx()) {
    gemmbitserial::GEMMContext ctx = mm->getCPUContext();
    gemmbitserial::gemmBitSerial(ctx);
#ifdef BISMORT_MATMUL_VERIFY_AGAINST_CPU
    mm->m_res->accel2host();
    size_t nbytes_res = mm->M()*mm->N()*sizeof(int32_t);
    int verify_res = memcmp(ctx.res, mm->m_res->hostbuf(), nbytes_res);
    std::cout << "CPU vs accel verification result = " << verify_res << std::endl;
#endif
  }
}

InstrumentationData getInstrumentationData(LayerHandle id) {
  MatrixMultiply * mm = (MatrixMultiply *) id;
  acc->updateStateBreakdown();
  mm->perfSummary();
  mm->perfDetails();
  return instrumentationData;
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
  if(mm->has_cpu_ctx()) {
    gemmbitserial::GEMMContext ctx = mm->getCPUContext();
    ctx.lhs.importRegular(mm->m_lhs->hostbuf());
  }
}

void syncLayerRHSBuffer(LayerHandle id) {
  MatrixMultiply * mm = (MatrixMultiply *) id;
  mm->m_rhs->host2accel();
  if(mm->has_cpu_ctx()) {
    gemmbitserial::GEMMContext ctx = mm->getCPUContext();
    ctx.rhs.importRegular(mm->m_rhs->hostbuf());
  }
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

}
