#ifndef BISMORT_SHARED_BUFFER_HPP
#define BISMORT_SHARED_BUFFER_HPP

#include <string>
#include "wrapperregdriver.h"

namespace bismo_inference {

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
    if(is_coherent) {
      // TODO use coherent allocation
    } else {
      m_hostbuf = new T[n_elems];
      m_accelbuf = (uint32_t) m_platform->allocAccelBuffer(nbytes());
    }
  };

  ~SharedBuffer() {
    m_platform->deallocAccelBuffer((void *) m_accelbuf);
    delete [] m_hostbuf;
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
        m_platform->copyBufferHost2Accel(
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

#endif BISMORT_SHARED_BUFFER_HPP
