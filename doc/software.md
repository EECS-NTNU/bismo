# Software

BISMO overlays are programmable via the instructions listed in the paper,
although full compiler support is lacking at this point.
A rudimentary software stack can be found under `src/main/cpp/app`, which
consists of the following:

* `BitSerialMatMulAccelDriver.hpp` is the low-level driver for BISMO, exposing
function calls to write instructions to BISMO queues, launching the accelerator,
and reading performance counters.
One level under this low-level driver is the register driver
`BitSerialMatMulAccel.hpp` which is generated automatically by
[fpga-tidbits
PlatformWrapper](https://github.com/maltanar/fpga-tidbits/wiki/platformwrapper).

* `BitSerialMatMulExecutor.hpp` contains a rudimentary high-level driver,
exposing function calls to generate instruction sequences corresponding to
bit-serial matrix multiplication. Currently this is limited to binary matrices,
you must manually construct the instruction sequences for multi-bit matrices.

* `BISMOTests.hpp` contains the top-level test code, which also serve as
usage examples. It uses `BitSerialMatMulExecutor` calls.

* `gemmbitserial`: To convert regular int8 matrices into a format suitable for
bit-serial, we use functions from
[gemmbitserial](https://github.com/maltanar/gemmbitserial).
Inputs to `BitSerialMatMulExecutor` must be in this format.
