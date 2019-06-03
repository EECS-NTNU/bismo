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
  MatrixMultiply * mm = new MatrixMultiply(lhs, rhs, res);

  return (LayerHandle) mm;
}

void execMatMulLayer(LayerHandle id) {
  MatrixMultiply * mm = (MatrixMultiply *) id;
  mm->exec();
#ifdef BISMORT_MATMUL_VERIFY_AGAINST_CPU
  std::cout << "Verification result = " << mm->verify() << std::endl;
#endif
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

}
