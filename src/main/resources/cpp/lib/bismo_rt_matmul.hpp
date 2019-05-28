#include "bismo_inference_internal.hpp"
#include "bismo_rt_matrix.hpp"

namespace bismo_inference {

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

  Matrix<uint8_t> * m_lhs, * m_rhs;
  Matrix<int32_t> * m_res;
protected:
  SingleMMDescriptor m_igen_dsc;
};

}
