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

#ifndef BISMORT_SHARED_BUFFER_HPP
#define BISMORT_SHARED_BUFFER_HPP

#include <string>
#include "wrapperregdriver.h"

namespace bismo_rt {

template<typename T>
class SharedBuffer {
public:
  // initialize a new SharedBuffer between host and accelerator
  SharedBuffer(
    size_t n_elems, // number of elements in buffer of type T
    WrapperRegDriver * platform,  // WrapperRegDriver for accel-side allocation
    std::string name = "", // buffer name, useful for debugging/stats
    bool is_const = false,   // constant buffer, copy only once
    bool is_coherent = false  // use coherency (needs special support)
  ) {
    m_n_elems = n_elems;
    m_is_coherent = is_coherent;
    m_platform = platform;
    m_name = name;
    m_is_host_dirty = true;
    m_is_const = is_const;
    m_accelbuf = (uint32_t)(uint64_t) m_platform->allocAccelBuffer(nbytes());
    if(m_is_coherent) {
      m_hostbuf = (T *) platform->phys2virt((void *) m_accelbuf);
    } else {
      m_hostbuf = new T[n_elems];
    }
  };

  ~SharedBuffer() {
    m_platform->deallocAccelBuffer((void *) m_accelbuf);
    if(!m_is_coherent) {
      delete [] m_hostbuf;
    }
  }

  // copy accel buffer to host buffer
  void accel2host() {
    if(!m_is_coherent) {
      // TODO add instrumentation (measure copy times)
      m_platform->copyBufferAccelToHost(
        (void *) m_accelbuf, (void *) m_hostbuf, nbytes()
      );
    }
  };

  // copy host buffer to accel buffer
  void host2accel() {
    if(!m_is_coherent) {
      if(m_is_const && !m_is_host_dirty) {
        // do nothing, already copied
      } else {
        // TODO add instrumentation (measure copy times)
        m_platform->copyBufferHostToAccel(
          (void *) m_hostbuf, (void *) m_accelbuf, nbytes()
        );
        m_is_host_dirty = false;
      }
    }
  };

  // return the number of bytes occupied in the host buffer
  const size_t nbytes() const {
    return m_n_elems * sizeof(T);
  };

  // read and compare the contents of the host and accel buffers
  // returns true if equal, false otherwise
  bool compare() {
    T * accelbuf_contents = new T[m_n_elems];
    m_platform->copyBufferAccelToHost(
      (void *) m_accelbuf, (void *) accelbuf_contents, nbytes()
    );
    bool ret = (memcmp(m_hostbuf, accelbuf_contents, nbytes()) == 0);
    delete [] accelbuf_contents;
    return ret;
  };

  // get a host-accessible pointer to the host buffer
  T * hostbuf() {
    return m_hostbuf;
  };

  // get an accel-accessible pointer to the accel buffer
  uint32_t accelbuf() {
    return m_accelbuf;
  };

protected:
  size_t m_n_elems;
  bool m_is_coherent;
  bool m_is_const;
  bool m_is_host_dirty;
  T * m_hostbuf;
  uint32_t m_accelbuf;
  std::string m_name;
  WrapperRegDriver * m_platform;
};

}

#endif /* BISMORT_SHARED_BUFFER_HPP */
