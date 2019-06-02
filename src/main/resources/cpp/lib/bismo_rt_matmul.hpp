#ifndef BISMORT_MATMUL_HPP
#define BISMORT_MATMUL_HPP

#include "bismo_rt_internal.hpp"
#include "bismo_rt_matrix.hpp"

namespace bismo_rt {

// an executable BISMO matrix multiply operation
// input and result matrices must be preallocated, see Matrix<>
class MatrixMultiply {
public:
  // construct a new BISMO matrix multiply operation from preallocated matrices
  // lhs = left hand side (e.g. MxK)
  // rhs = right hand side (e.g. KxN, must be transposed)
  // res = result (e.g. MxN, must be transposed)
  MatrixMultiply(
    Matrix<uint8_t> * lhs, Matrix<uint8_t> * rhs, Matrix<int32_t> * res
  );
  // free matrix multiply operation, does NOT free input/output matrices
  ~MatrixMultiply();
  // execute matrix multiply
  // does not synchronize input Matrix objects, remember to call host2accel
  void exec();
  // verify the computed matmul result against a CPU-computed result
  int verify();
  // convenience function to get matmul dimensions
  size_t M() const;
  size_t K() const;
  size_t N() const;
  // performance / instrumentation related functions
  size_t lhsBytes() const;
  size_t rhsBytes() const;
  size_t resBytes() const;
  size_t getNumBytesToFetch() const;
  size_t getNumBytesToWrite() const;
  float getWorkloadOpCount(bool inclPadding = true) const;
  float getWorkloadBinaryOpCount(bool inclPadding = true) const;
  float getLastRunBinaryGOPS(bool inclPadding = true) const;
  float getWorkloadReadOI() const;
  float getWorkloadWriteOI() const;
  float getActualReadOI() const;
  float getActualWriteOI() const;
  float getWorkloadOI() const;
  // get performance summary and details (saved into bismo_rt::instrumentationData)
  void perfSummary();
  void perfDetails();
  // lhs/rhs/res member matrices, exposed for the sake of the API wrapper
  Matrix<uint8_t> * m_lhs, * m_rhs;
  Matrix<int32_t> * m_res;
protected:
  SingleMMDescriptor m_igen_dsc;
};

}

#endif /* end of include guard: BISMORT_MATMUL_HPP */
