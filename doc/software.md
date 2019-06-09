# Software

BISMO is a hardware-focused project for the most part, but comes with a minimal software stack to make it easier to use.

## The BISMO runtime library and API

The BISMO runtime library is the preferred way of using BISMO. It exposes a
simple API that enables the user to create matrix multiplication operations
of specified dimensions and bitwidth, fill the matrices with desired data,
perform the execution, retrieve the result and read out instrumentation data.
It abstracts the user from the details of accelerator/host data movement and
the hardware/software interface.

**How do I use this?**
Each platform provides a script that compiles the runtime library into a
shared object `libbismo_rt.so` as part of the deployment folder.
The public API for the BISMO runtime library can be found in
`src/main/resources/lib/bismo_rt.hpp`, and is copied to `rtlib/bismo_rt.hpp`
in the deployment folder. Simply include the header file in your application,
and link to the shared library.


**What is the API?** Here is a brief explanation of what you'll find in the
BISMO RT. Note that all of this is under the bismo_rt namespace, so you'll
have to add either `using namespace bismo_rt;` or prefix each call with
`bismo_rt::`. You can find more detailed descriptions in `bismo_rt.hpp`.

| Function() or *Type*      | Description       | Parameters  | Returns |
| ------------- |:-------------:| -----:| -----:|
| *MatMulDescriptor*      | A struct that describes the dimensions for a matrix multiply operation | number of bits, signedness, spatial matrix size | n/a |
| *LayerHandle*      | Identifies an instantiated BISMO matrix multiply operation | n/a | n/a |
| *InstrumentationData*      | An `std::map<std::string,float>` that contains name-value pairs for instrumentation data. | n/a | n/a |
| *HardwareConfig*      | A struct that contains the instantiated BISMO overlay configuration. | n/a | n/a |
| init()      | Initializes the hardware and runtime library, call this before calling anything else | none | none |
| initMatMul()      | Create a matrix multiply operation | MatMulDescriptor | LayerHandle |
| getLayerLHSBuffer()      | Get the host-accessible **row-major** buffer for left-hand-side (LHS) matrix in matrix multiply | LayerHandle | uint8_t * |
| getLayerRHSBuffer()      | Get the host-accessible **col-major** buffer for right-hand-side (RHS) matrix in matrix multiply | LayerHandle | uint8_t * |
| getLayerResBuffer()      | Get the host-accessible **col-major** buffer for the result matrix in matrix multiply | LayerHandle | int32_t * |
| syncLayerLHSBuffer()      | Ensure that the accelerator has an up-to-date version of the LHS matrix | LayerHandle | none |
| syncLayerRHSBuffer()      | Ensure that the accelerator has an up-to-date version of the RHS matrix | LayerHandle | none |
| syncLayerResBuffer()      | Ensure that the accelerator has an up-to-date version of the result matrix | LayerHandle | none |
| execMatMul()      | Execute a matrix multiply operation | LayerHandle | none |
| deinitMatMul()      | Free up resources used by a matrix multiply operation | LayerHandle | none |
| getInstrumentationData()      | Get the instrumentation data for the last executed matrix multiply | LayerHandle | InstrumentationData |
| getHardwareConfig()      | Retrieve hardware configuration for the BISMO instance | none | HardwareConfig |
| benchmark_host_accel_transfer()      | Benchmark host<->accel data transfer times | none | none |
| selftest_*()      | Various small self-test functions | none | none |
| deinit()      | De-initializes the hardware and runtime library | none | none |

**Is there any example code?** Check out the [top-level test](testing.md).

**How do you handle variable-bit datatypes?** To keep the API simple, we
currently use uint8_t as a container datatype, and specify the actual bitwidths
in the MatMulDescriptor when creating a matrix multiply operation.


**Are there any limitations on matrix sizes?** Yes, as with any computer system.
The restrictions you are most likely to run into are 1) bitwidth limitation (max 8 bits)
due to how the API is structured, and 2) matrix size limitation depending on
whether one stripe of the matrix fits into on-chip memory (this depends on the
overlay configuration, see the size checks in `src/main/resources/lib/bismo_rt_matmul.cpp`)
due to the current tiling strategy.
Even if you develop your own tiling, the amount of contiguous memory
available (determined by the platform) will also limit the maximum size.

**Is the API thread-safe?** Not at the moment, contributions to fix this are welcome.

## Under the Hood

This section is quite thin on content at the moment, contributions are welcome!

### The runtime

The runtime is spread across several files under
`src/main/resources/lib/` all prefixed with `bismo_rt_`. Most of the runtime
is a thin wrapper around the functionality already provided by the low-level
driver, plus some "added value" for instrumentation and data movement.

### The low-level driver

Internally, the runtime library uses the BISMO low-level driver to perform
various tasks. The low-level driver for BISMO, exposes function calls to write
instructions or matrix multiply descriptors to BISMO queues, launching the accelerator,
and reading performance counters.
The low-level driver can be found in `src/main/resources/lib/BitSerialMatMulAccelDriver.hpp`.

### The register driver

One level under this low-level driver is the register driver
`BitSerialMatMulAccel.hpp` which is generated automatically by
[fpga-tidbits
PlatformWrapper](https://github.com/maltanar/fpga-tidbits/wiki/platformwrapper).
You can find this file under `$BUILD_DIR/hw/driver`.
