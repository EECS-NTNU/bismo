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
  // gemmbitserial = allow pure CPU execution using gemmbitserial
  MatrixMultiply(
    Matrix<uint8_t> * lhs, Matrix<uint8_t> * rhs, Matrix<int32_t> * res,
    bool allow_gemmbitserial = false
  );
  // free matrix multiply operation, does NOT free input/output matrices
  ~MatrixMultiply();
  // execute matrix multiply on accelerator
  // does not synchronize input Matrix objects, remember to call host2accel
  void exec();
  // whether CPU-only execution is enabled
  bool has_cpu_ctx() const;
  // return gemmbitserial handle for CPU-only execution
  gemmbitserial::GEMMContext getCPUContext();
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
  gemmbitserial::GEMMContext m_cpu_ctx;
  bool m_allow_gemmbitserial;
};

}

#endif /* end of include guard: BISMORT_MATMUL_HPP */
