#ifndef BISMORT_MATRIX
#define BISMORT_MATRIX

#include "bismo_inference_internal.hpp"
#include "bismo_rt_shared_buffer.hpp"

namespace bismo_inference {

template<typename T>
class Matrix {
public:
  Matrix(
    // matrix dimensions
    size_t rows, size_t cols,
    // number of maximum bits that represent each element
    size_t bits,
    // whether the most significant
    bool is_signed,
    // whether the data is stored in col-major order
    // this is required for the rhs and result matrices in a matrix multiply
    // operation, due to the assumptions that BISMO makes
    bool is_transposed
  ) {
    m_rows = rows;
    m_cols = cols;
    m_bits = bits;
    m_is_signed = is_signed;
    m_is_transposed = is_transposed;
    const size_t outer_align = is_transposed ? cfg.dpaDimRHS : cfg.dpaDimLHS;
    const size_t inner_align = cfg.dpaDimCommon;
    m_outer_a = gemmbitserial::alignTo(outer(), outer_align);
    m_inner_a = gemmbitserial::alignTo(inner(), inner_align);
    // TODO support naming, constant matrices and coherency here
    m_padded_buf = new SharedBuffer<T>(
      elems_a(), platform, "", false, false
    );
    m_needs_padding = (outer() != outer_a()) || (inner() != inner_a());
    if(m_needs_padding) {
      m_unpadded_hostbuf = new T[elems()];
    } else {
      m_unpadded_hostbuf = 0;
    }
  };

  ~Matrix() {
    delete m_padded_buf;
    if(m_needs_padding) {
      delete [] m_unpadded_hostbuf;
    }
  }

  const size_t outer() const {
    return m_is_transposed ? m_cols : m_rows;
  }

  const size_t inner() const {
    return m_is_transposed ? m_rows : m_cols;
  }

  const size_t outer_a() const {
    return m_outer_a;
  }

  const size_t inner_a() const {
    return m_inner_a();
  }

  const size_t elems() const {
    return inner() * outer();
  }

  const size_t elems_a() const {
    return inner_a() * outer_a();
  }

  // copy accel buffer to host buffer
  void accel2host() {
    m_padded_buf.accel2host();
    if(m_needs_padding) {
      // TODO strided copy into m_unpadded_hostbuf
    }
  };

  // copy host buffer to accel buffer
  void host2accel() {
    if(m_needs_padding) {
      // TODO strided copy from m_unpadded_hostbuf
    }
    m_padded_buf.accel2host();
  };

  // get a host-accessible pointer to the host buffer
  T * hostbuf() {
    if(m_needs_padding) {
      return m_unpadded_hostbuf;
    } else {
      return m_needs_padding.hostbuf();
    }
  };

  // get an accel-accessible pointer to the accel buffer
  uint32_t accelbuf() {
    return m_padded_buf.accelbuf();
  };

protected:
  bool m_needs_padding;
  SharedBuffer<T> * m_padded_buf;
  T * m_unpadded_hostbuf;
  size_t m_rows, m_cols, m_bits;
  size_t m_inner_a, m_outer_a;
  bool m_is_signed;
  bool m_is_transposed;
};

}
#endif /* end of include guard: BISMORT_MATRIX */
