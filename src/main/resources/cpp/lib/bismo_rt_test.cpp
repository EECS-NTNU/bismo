#include "bismo_inference_internal.hpp"
#include "bismo_rt_shared_buffer.hpp"

namespace bismo_inference {

// run a simple self-test on the SharedBuffer functionality
bool selftest_shared_buffer() {
  string test_name = "selftest_shared_buffer";
  cout << "Starting test:" << test_name << endl;
  size_t n_elems = 10;
  SharedBuffer<int32_t> * buf = new SharedBuffer<int32_t>(
    10, platform, "test_buf", false, false
  );
  int32_t * host_buf = buf->hostbuf();
  for(size_t i = 0; i < n_elems; i++) {
    host_buf[i] = i+1;
  }
  bool all_ok = true;
  // expect host-accel contents to be initially different
  // TODO this assumes no coherency
  all_ok &= (buf->compare() == false);
  // after host-accel sync they should be the same again
  buf->host2accel();
  all_ok &= (buf->compare() == true);
  cout << "Test result = " << all_ok << endl;
  return all_ok;
}

}
