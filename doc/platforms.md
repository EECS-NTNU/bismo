# Platforms

BISMO can be deployed on a variety of boards using Xilinx FPGAs using the
same hardware and software.
You can specify which platform to use by using the `PLATFORM` environment
variable e.g. `PLATFORM=PYNQU96 make all`.

## List of supported platforms

| PLATFORM        | Board       | Remarks  | Largest configuration  |
| ------------- |:-------------:| -----:| -----:|
| VerilatedTester      | none (emulated) | Emulates 64-bit fixed-latency DRAM | depends on your host system |
| PYNQZ1      | Xilinx PYNQ-Z1 | Uses 64-bit AXI HP0 port | 8x256x8@200 MHz: 6.5 binary TOPS |
| PYNQU96      | Avnet Ultra96 |  Uses 64-bit AXI HP0 port | 10x256x10@300 MHz: 15 binary TOPS |
| PYNQU96CC | Avnet Ultra96 | (experimental) Support for coherent memory using 64-bit HPC0 port | 10x256x10@300 MHz: 15 binary TOPS |

## Using VerilatedTester for emulation

The VerilatedTester is different than other platforms; it does not create an
FPGA bitfile, but rather an executable that provides a cycle-accurate model of
BISMO. This is accomplished by using [Verilator](https://www.veripool.org/wiki/verilator)
to translate the generated Verilog into a cycle-accurate C++ model.
The hardware/software interface towards BISMO does not change and the same
runtime is used, although a different register-level driver is used under
the hood to map register read/writes and host/accel memory copies to C++ model
calls instead. The resulting emulation can be instrumented to a great level of
detail, including the monitoring of cycle-by-cycle behavior by waveform
dumping or adding printf statements inside Chisel. This allows evaluating
any hardware changes with greater debug capability and rapid design iterations.

## Adding support for a new platform

BISMO uses the the [fpga-tidbits
PlatformWrapper](https://github.com/maltanar/fpga-tidbits/wiki/platformwrapper)
infrastructure to deploy the same RTL and software on different platforms.
In order to add a new platform, follow these steps:

1. Add support for desired platform to
[fpga-tidbits](https://github.com/maltanar/fpga-tidbits)
and send a pull request on fpga-tidbits. For platforms that resemble existing
ones (e.g. another Zynq UltraScale+ running PYNQ) this will be mostly a
copy-paste of existing files, changing only the FPGA part number in the
`TidbitsMakeUtils.fpgaPartMap`.

2. Create a new folder with the same name as the platform for BISMO under
`src/main/script`. This should have two subfolders `host` and `target`, where
`host` contains the host-side bitfile generation scripts and `target` provides
a way of compiling the BISMO driver on the target platform. For platforms that
resemble existing ones, copy-pasting one of the existing platform script
folders is a good starting point.

3. Finally, add a Makefile include (.mk) under `platforms` with the same name as
the platform. This describes how to build the various hardware and software
components for the platform. See existing .mk files to get started.
