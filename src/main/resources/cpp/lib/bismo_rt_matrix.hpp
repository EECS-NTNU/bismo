// Copyright (c) 2019 Xilinx
//
// BSD v3 License
//
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// * Redistributions of source code must retain the above copyright notice, this
//   list of conditions and the following disclaimer.
//
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
//
// * Neither the name of BISMO nor the names of its
//   contributors may be used to endorse or promote products derived from
//   this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#ifndef BISMORT_MATRIX_HPP
#define BISMORT_MATRIX_HPP

#include <iostream>
#include <string>
#include "bismo_rt_internal.hpp"
#include "bismo_rt_shared_buffer.hpp"

namespace bismo_rt {

typedef enum {
  matTypeLHS, matTypeRHS, matTypeRes
} MatrixType;

template<typename T>
class Matrix {
public:
  Matrix(
    // matrix dimensions
    size_t rows, size_t cols,
    // number of maximum bits that represent each element
    size_t bits,
    // whether the number is signed (MSB represents a negative power-of-two)
    bool is_signed,
    // whether the data is stored in col-major order
    // this is required for the rhs and result matrices in a matrix multiply
    // operation, due to the assumptions that BISMO makes
    bool is_transposed,
    // matrix type, needed to align correctly for BISMO hardware
    MatrixType matrix_type,
    // matrix name, useful for debugging and instrumentation
    std::string name = "",
    // assume cache-coherent memory between CPU and accelerator
    bool is_coherent = false
  ) {
    m_is_coherent = is_coherent;
    m_name = name;
    m_matrix_type = matrix_type;
    m_rows = rows;
    m_cols = cols;
    m_bits = bits;
    m_is_signed = is_signed;
    m_is_transposed = is_transposed;
    /* Summary of alignment, transposition, datatype requirements:
      MatType   Transpose?  OuterAlign    InnerAlign  Dtype
      LHS       false       Dm            Dk          u/int8
      RHS       true        Dn            Dk          u/int8
      res       true        Dn            Dm          int32
    */
    if(matrix_type == matTypeLHS) {
      if(is_transposed) throw "LHS matrix must be non-transposed";
      if(sizeof(T) != 1) throw "LHS matrix must use 8-bit datatype";
    }
    if(matrix_type == matTypeRHS) {
      if(!is_transposed) throw "RHS matrix must be transposed";
      if(sizeof(T) != 1) throw "RHS matrix must use 8-bit datatype";
    }
    if(matrix_type == matTypeRes) {
      if(!is_transposed) throw "Result matrix must be transposed";
      if(sizeof(T) != 4 || !m_is_signed) throw "Result matrix must use int32 datatype";
    }
    const size_t outer_align = is_transposed ? cfg.dpaDimRHS : cfg.dpaDimLHS;
    const size_t inner_align = matrix_type == matTypeRes ? cfg.dpaDimLHS : cfg.dpaDimCommon;
    m_outer_a = gemmbitserial::alignTo(outer(), outer_align);
    m_inner_a = gemmbitserial::alignTo(inner(), inner_align);
    // TODO support constant matrices
    m_padded_buf = new SharedBuffer<T>(
      elems_a(), platform, m_name+"_buf", false, is_coherent
    );
    m_needs_padding = (outer() != outer_a()) || (inner() != inner_a());
    if(m_needs_padding) {
      m_unpadded_hostbuf = new T[elems()];
      // initialize the padded buf to all zeroes
      // using memset here throws "Bus error" so using a for-loop instead:
      // memset(padded_hostbuf(), 0, elems_a() * sizeof(T));
      for(size_t i = 0; i < elems_a(); i++) {
        padded_hostbuf()[i] = 0;
      }
    } else {
      m_unpadded_hostbuf = 0;
    }
    m_is_bitserial = (matrix_type != matTypeRes);
    if(is_bitserial()) {
      m_bitserial_accelbuf = (uint32_t)(uint64_t) platform->allocAccelBuffer(bitserial_nbytes());
    }
  };

  ~Matrix() {
    delete m_padded_buf;
    if(m_needs_padding) {
      delete [] m_unpadded_hostbuf;
    }
    if(is_bitserial()) {
      platform->deallocAccelBuffer((void *) m_bitserial_accelbuf);
    }
  };

  void printSummary() {
    std::cout << "Matrix: " << std::endl;
    std::cout << m_rows << " x " << m_cols << ":" << m_bits << "b" << std::endl;
    std::cout << "Signed? " << m_is_signed << " transposed? " << m_is_transposed << std::endl;
    std::cout << "Mode: " << m_matrix_type << " padding? " << m_needs_padding << std::endl;
    std::cout << "Outer x inner: " << outer() << " x " << inner() << std::endl;
    std::cout << "Aligned outer x inner: " << outer_a() << " x " << inner_a() << std::endl;
  };

  const bool is_bitserial() const {
    return m_is_bitserial;
  }

  const size_t bits() const {
    return m_bits;
  };

  const bool is_signed() const {
    return m_is_signed;
  };

  const size_t outer() const {
    return m_is_transposed ? m_cols : m_rows;
  };

  const size_t inner() const {
    return m_is_transposed ? m_rows : m_cols;
  };

  const size_t outer_a() const {
    return m_outer_a;
  };

  const size_t inner_a() const {
    return m_inner_a;
  };

  const size_t elems() const {
    return inner() * outer();
  };

  const size_t elems_a() const {
    return inner_a() * outer_a();
  };

  // copy accel buffer to host buffer
  void accel2host() {
    TIMER_SAMPLE();
    m_padded_buf->accel2host();
    TIMER_SAMPLE();
    TIMER_REPORT(m_name + "_accel2host");
    TIMER_SAMPLE();
    if(m_needs_padding) {
      // strided copy into m_unpadded_hostbuf
      copy2d(
        m_padded_buf->hostbuf(), m_unpadded_hostbuf,
        outer_a(), inner_a(), outer(), inner()
      );
      // TODO time measurement for un-padding
    }
    TIMER_SAMPLE();
    TIMER_REPORT(m_name + "_unpad");
  };

  // copy host buffer to accel buffer
  void host2accel() {
    TIMER_SAMPLE();
    if(m_needs_padding) {
      // strided copy from m_unpadded_hostbuf
      copy2d(
        m_unpadded_hostbuf, m_padded_buf->hostbuf(),
        outer(), inner(), outer_a(), inner_a()
      );
      // TODO time measurement for padding
    }
    TIMER_SAMPLE();
    TIMER_REPORT(m_name + "_pad");
    TIMER_SAMPLE();
    m_padded_buf->host2accel();
    TIMER_SAMPLE();
    TIMER_REPORT(m_name + "_host2accel");
    TIMER_SAMPLE();
    if(is_bitserial()) {
      // TODO call only once for const matrices
      p2s();
    }
    TIMER_SAMPLE();
    TIMER_REPORT(m_name + "_p2s");
  };

  // get a host-accessible pointer to the host buffer
  T * hostbuf() {
    if(m_needs_padding) {
      return m_unpadded_hostbuf;
    } else {
      return m_padded_buf->hostbuf();
    }
  };

  // get a host-accessible pointer to the padded host buffer
  T * padded_hostbuf() {
    return m_padded_buf->hostbuf();
  };

  // get an accel-accessible pointer to the bit-parallel accel buffer
  uint32_t accelbuf() {
    return m_padded_buf->accelbuf();
  };

  uint32_t bitserial_accelbuf() {
    if(is_bitserial()) {
      return m_bitserial_accelbuf;
    } else {
      throw "Cannot access bitserial accelbuf for non-bitserial matrix.";
    }
  }

  size_t bitserial_nbytes() const {
    if(is_bitserial()) {
      return (m_bits * elems_a()) / 8;
    } else {
      throw "Cannot get bitserial buffer size for non-bitserial matrix.";
    }
  }

  size_t elem_nbytes() const {
    return sizeof(T);
  }

  // convert the accelerator bit-parallel buffer to bit-serial
  uint32_t p2s() {
    if(!m_is_bitserial) {
      throw "Unsupported matrix type for parallel-to-serial conversion.";
    }
    // setup and call the p2s hardware accelerator
    acc->setup_p2s(
      (void *) accelbuf(),  // source buffer
      bitserial_nbytes(),   // num bytes to be written to dest
      (void *) bitserial_accelbuf(),  // dest buffer
      outer_a(), inner_a(), bits(),   // dimensions
      is_signed()
    );
    uint32_t cycles = acc->p2s_exec_and_wait();
    return cycles;
  }

  // two-dimensional memory copy between arrays of different
  // dimensions, useful for padding and un-padding
  static void copy2d(
    T * src, T * dst, // source and destination host buffers
    size_t src_n_outer, size_t src_n_inner, // source dims
    size_t dst_n_outer, size_t dst_n_inner  // destination dims
  ) {
    const size_t min_outer = std::min(src_n_outer, dst_n_outer);
    const size_t min_inner = std::min(src_n_inner, dst_n_inner);
    for(size_t o = 0; o < min_outer; o++) {
      // using memcpy here can cause bus errors with coherency enabled,
      // so opting for a secondary for-loop instead
      // memcpy(dst, src, sizeof(T) * min_inner);
      for(size_t i = 0; i < min_inner; i++) {
        dst[i] = src[i];
      }
      dst += dst_n_inner;
      src += src_n_inner;
    }
  };

protected:
  bool m_needs_padding;
  uint32_t m_bitserial_accelbuf;
  SharedBuffer<T> * m_padded_buf;
  T * m_unpadded_hostbuf;
  size_t m_rows, m_cols, m_bits;
  size_t m_inner_a, m_outer_a;
  bool m_is_signed;
  bool m_is_transposed;
  bool m_is_bitserial;
  MatrixType m_matrix_type;
  std::string m_name;
  bool m_is_coherent;
};

}
#endif /* end of include guard: BISMORT_MATRIX_HPP */
