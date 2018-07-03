# BISMO

BISMO is a programmable FPGA accelerator for few-bit integer matrix multiplication.
It offers high-performance matrix multiplication for matrices where each
element is a few-bit integer (e.g. 2, 3, 4 ... bits).
This is beneficial for applications like
[quantized neural network inference](https://arxiv.org/abs/1709.04060)
and
[approximate computing](https://en.wikipedia.org/wiki/Approximate_computing)
approaches.


Its key features are:
* **High performance and energy efficiency.** On the
  [Xilinx PYNQ-Z1 board](http://www.pynq.io/board.html), BISMO can
  offer 6.5 TOPS of binary matrix multiplication performance while drawing less
  than 5 W of power.
* **Configurable size.** The hardware can be scaled up for higher performance, or
  down to save on FPGA resources and power consumption.
* **Runtime scales with precision.** The input matrices can have any number of
  bits specified at runtime. Higher bit-precision will take more time.
* **Software-programmable.** BISMO is programmable with a simple instruction set to
  cater for different matrix sizes, precisions.

## Paper
More details on the hardware design and instruction set can be found in the
[BISMO paper](https://arxiv.org/abs/1806.08862).
If you find BISMO useful, please use the following citation:
```
@inproceedings{bismo,
author = {Umuroglu, Yaman and Rasnayake, Lahiru and Sjalander, Magnus},
title = {BISMO: A Scalable Bit-Serial Matrix Multiplication Overlay for Reconfigurable Computing},
booktitle = {Field Programmable Logic and Applications (FPL), 2018 28th International Conference on},
series = {FPL '18},
year = {2018}
}
```

## Requirements
1. A working [`sbt`](https://www.scala-sbt.org/1.0/docs/Installing-sbt-on-Linux.html) setup for Chisel2
2. `zsh` e.g. `sudo apt install zsh` on Ubuntu
3. [Vivado 2017.4](https://www.xilinx.com/support/download.html) (make sure `vivado` is in `PATH`)
4. `gcc` 4.8 or later
4. A [Xilinx PYNQ-Z1 board](http://www.pynq.io/board.html) board with the v1.4 image or later, with network access

## Installation
1. `git clone --recurse-submodules https://github.com/EECS-NTNU/bismo.git`

The `--recurse-submodules` option fetches the git repos that BISMO depends on,
which are `fpga-tidbits`, `oh-my-xilinx` and `gemmbitserial`.
If you forget to pass this option while cloning, you can `cd` into the repo
root and run `git submodule init && git submodule update` instead.

## Quickstart

BISMO is primarily a "hardware library" in that it generates a bit-serial matrix
multipliciation accelerator with specified dimensions.
However, it also contains software code that acts both as a usage example and
also as a test suite.
The same software code is used in both cases, using the [fpga-tidbits
PlatformWrapper](https://github.com/maltanar/fpga-tidbits/wiki/platformwrapper)
infrastructure.

You can "run" BISMO with this code either in hardware-software cosimulation
on a host PC, or on the actual FPGA platform as follows:

### Running HW-SW Cosimulation
1. `cd bismo`
2. `make emu` to run BISMO tests in hardware-software cosimulation.

### Run Tests on PYNQ-Z1
On the host computer:
1. `cd bismo`
2. `make all` to generate a PYNQ-Z1 deployment package with bitfile and drivers.
This will generate a 8x256x8 array at 200 MHz and will take some time to complete.
3. Set `PYNQZ1_URI` to point to the `rsync` target, including the username, IP
address and target directory on the PYNQ-Z1 board.
For instance `export PYNQZ1_URI=xilinx@192.168.2.10:/home/xilinx/bismo`
4. `make rsync` to copy deployment package to the PYNQ-Z1. You may be prompted
for the password for the specified PYNQ-Z1 user.

Afterwards, run the following on a terminal on the PYNQ-Z1:
1. On the PYNQ-Z1, `cd /home/xilinx/bismo/deploy` to go into the deployment package.
2. `./compile_sw.sh` to compile the driver and tests.
3. `sudo ./setclk.sh 200` to set the clock to 200 MHz.
4. `sudo ./load_bitfile.sh` to load the BISMO bitfile.
5. `sudo ./app` to run the BISMO tests.

## Hardware
BISMO is implemented in [Chisel 2](https://chisel.eecs.berkeley.edu) using
components from the [fpga-tidbits](https://github.com/maltanar/fpga-tidbits/)
framework, and targets Xilinx FPGAs.
The Chisel source code can be found under `src/main/scala`.
Currently there is no separate documentation available for the hardware design,
the best sources of information are the [BISMO paper](https://arxiv.org/abs/1806.08862) 
and the comments in the code.

### Overlay Configuration
BISMO is parametrized and can be instantiated in different sizes to generate a
higher-performance overlay using more FPGA resources.

*Directly in the source code:* The actual instantiaton is carried out in
`ChiselMain.main` defined in `src/main/scala/Main.scala` which uses the
`BitSerialMatMulParams` class defined in `src/main/scala/BISMO.scala` to specify
the overlay configuration.
You can specify the overlay dimensions directly in `ChiselMain.main`.

*As environment variables (limited):* For quick experimentation, three of the
overlay dimensions are sourced from environment variables when `make` is called.
The environment variables are `M (dpaDimLHS)`, `K (dpaDimCommon)` and
`N (dpaDimRHS)`.
For instance, `export M=2 K=64 N=2; make all` will generate a 2x64x2 overlay.

*Special note for hardware-software cosimulation:* The overlay configuration for
hardware-software cosimulation is specified separately in
`Settings.emuInstParams` under `src/main/scala/Main.scala`.
Note that running cosimulation for large instances may take a long time.

### Resource Characterization Flow
You can run the characterization flow to see how the FPGA resource usage
varies with configuration for different BISMO components.
To run the characterization, simply use the name defined under
`CharacterizeMain.main` as a `make` target, e.g. `make CharacterizePC` will
run the characterization for the PopCountUnit.
This will produce a file `CharacterizePC.log` with the results.
Completing the characterization will take some time depending on the range of
parameters.
To change the range of parameters for characterization, see the
`makeParamSpace` functions under `CharacterizeMain`.

## Software

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


## More Tests
Besides the top-level BISMO test described under Quickstart, there are several
other tests available for BISMO components under `src/test`.
See [here](src/test/scala) for more on the pure Scala/Chisel tests, and
[here](src/test/cosim) for more on the smaller cosimulation tests.
