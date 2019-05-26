#ifndef BISMORT_MATRIX_HPP
#define BISMORT_MATRIX_HPP
#include "bismo_inference_internal.hpp"

namespace bismo_inference {

void matrix_accel2host(MatrixHandle mat) {
  MatrixDescriptor dsc = mat_registry[mat];
  if(dsc.is_coherent) {
    throw "Coherency not yet suported for matrix_accel2host";
  } else {
    // get rid of padding as needed
    if(!dsc.has_padding()) {
      // no padding was necessary, copy directly
      platform->copyBufferAccelToHost(dsc.accelbuf, dsc.hostbuf, dsc.nbytes_accel());
    } else {
      if(!dsc.is_int32) {
        // otherwise this would require bit serial -> bit parallel...
        throw "matrix_accel2host only suports int32_t matrices for now.";
      }
      // accelerator has a padded result matrix, copy actual parts only
      size_t elem_size = dsc.is_int32 ? sizeof(int32_t) : sizeof(uint8_t);
      size_t nbytes_nonpadded = elem_size * dsc.n_inner();
      for(size_t outer = 0; outer < dsc.n_outer(); outer++) {
        size_t ind_padded = outer * dsc.n_inner_a() * elem_size;
        size_t ind_actual = outer * dsc.n_inner() * elem_size;
        platform->copyBufferAccelToHost(
          (void *)(dsc.accelbuf + ind_padded),
          (void *)((uint64_t)dsc.hostbuf + ind_actual), nbytes_nonpadded
        );
      }
    }
  }
}

void matrix_host2accel(MatrixHandle mat) {
  MatrixDescriptor dsc = mat_registry[mat];
  if(dsc.is_coherent) {
    throw "Coherency not yet suported for matrix_host2accel";
    // note: this will need a p2s_accel2accel and p2s_host2accel separation,
    // as well as an additional accel buffer in the matrix descriptor
  } else {
    if(dsc.is_int32) {
      throw "matrix_accel2host only suports non-int32_t matrices for now.";
    } else {
      p2s(
        (const uint8_t *)dsc.hostbuf, dsc.accelbuf, dsc.n_outer(), dsc.n_inner(),
        dsc.bits, dsc.is_signed, false,
        dsc.is_transposed ? cfg.dpaDimRHS : cfg.dpaDimLHS
      );
    }
  }
}

MatrixHandle matrix_create(
  size_t rows, size_t cols, size_t bits, bool is_signed, bool is_transposed,
  bool is_int32
) {
  // we make some assumptions about int32 matrices in the rtlib,
  // make sure they are respected
  if(is_int32 && !is_signed) {
    throw "int32 matrices must have is_signed = true";
  }
  if(is_int32 && !is_transposed) {
    throw "int32 matrices must have is_transposed = true";
  }
  if(is_int32 && bits != 32) {
    throw "int32 matrices must have bits = 32";
  }
  MatrixDescriptor dsc;
  dsc.rows = rows;
  dsc.cols = cols;
  dsc.bits = bits;
  dsc.is_signed = is_signed;
  dsc.is_transposed = is_transposed;
  dsc.is_int32 = is_int32;
  // use is_transposed to determine what to align to
  size_t row_align = is_transposed ? cfg.dpaDimRHS : cfg.dpaDimLHS;
  dsc.rows_a = gemmbitserial::alignTo(rows, row_align);
  dsc.cols_a = gemmbitserial::alignTo(cols, cfg.dpaDimCommon);
  // TODO set coherency according to availability in platform instead
  dsc.is_coherent = false;
  if(dsc.is_coherent) {
    // TODO do coherent allocation for host/accel buffers
    // TODO can't simply keep bit-par host and bit-ser accel buf coherent..
  } else {
    dsc.accelbuf = (uint32_t) platform->allocAccelBuffer(dsc.nbytes_accel());
    // allocate hostbuf according to type
    if(dsc.is_int32) {
      dsc.hostbuf = (void *) new int32_t[rows * cols];
    } else {
      dsc.hostbuf = (void *) new uint8_t[rows * cols];
    }
  }
}

void * matrix_get_hostbuf(MatrixHandle mat) {
  MatrixDescriptor dsc = mat_registry[mat];
  return dsc.hostbuf;
}

void matrix_destroy(MatrixHandle mat) {
  MatrixDescriptor dsc = mat_registry[mat];
  if(dsc.is_coherent) {
    // TODO deallocate coherent buffers
  } else {
    platform->deallocAccelBuffer(dsc.accelbuf);
    // deallocate hostbuf according to type
    if(dsc.is_int32) {
      delete [] (int32_t*) dsc.hostbuf;
    } else {
      delete [] (uint8_t*) dsc.hostbuf;
    }
  }
}

};
#endif // BISMORT_MATRIX_HPP
