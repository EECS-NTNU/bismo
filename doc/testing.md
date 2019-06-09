# Testing

## Top-level test
BISMO is primarily a library and not an application. However, there is a
top-level application that serves as both a test suite and an example application.
The source code for this application can be found under
`src/main/resources/cpp/app`. It links against the BISMO runtime library,
which may be running on top of actual hardware or a cycle-accurate C++
emulation model on the host. It takes a single command line parameter
indicating which mode to run in:

* `t` to run the test suite
* `i` to run interactive benchmarking
* `b` to run batch-mode benchmarking

## Unit tests
Besides the top-level BISMO test described here, there are several
other tests available for BISMO components under `src/test`.
See [here](src/test/scala) for more on the pure Scala/Chisel tests, and
[here](src/test/cosim) for more on the smaller cosimulation tests.
