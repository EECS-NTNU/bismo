# Platforms

BISMO can be deployed on a variety of boards using Xilinx FPGAs using the
same hardware and software.
You can specify which platform to use by using the `PLATFORM` environment
variable e.g. `PLATFORM=PYNQU96 make all`.

## List of supported platforms

| PLATFORM        | Board       | Remarks  | Largest configuration  |
| ------------- |:-------------:| -----:| -----:|
| PYNQZ1      | Xilinx PYNQ-Z1 | Uses 64-bit AXI HP0 port | 8x256x8@200 MHz: 6.5 binary TOPS |
| PYNQU96      | Avnet Ultra96 |  Uses 64-bit AXI HP0 port | 10x256x10@300 MHz: 15 binary TOPS |
| PYNQU96CC | Avnet Ultra96 | (experimental) Support for coherent memory using 64-bit HPC0 port | 10x256x10@300 MHz: 15 binary TOPS |

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
