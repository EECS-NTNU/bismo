# Platforms, Build and Deployment

BISMO assumes that the hardware design and supporting files are first prepared
on a *host*, and then deployed on a *target*. These may be the same computer in
some cases.

## Deployment

In general, each supported platform may do deployment slightly differently,
but all current BISMO platforms do this in a similar manner:

1. All necessary files (including bitfile, software drivers, runtime and top-level test application) are generated and put into a deployment folder.
2. The deployment folder is copied onto the target device (we assume using `rsync`)
3. User runs scripts for compiling runtime library and test application on the target device.
4. User flashes the bitfile
5. User runs the test application (or their own application)

All generated files during build are placed under a directory whose path
reflects the current platform and configuration, see `BUILD_DIR` in Makefile
variables below. The deployment folder is generated at `$BUILD_DIR/deploy`.

## Useful Make targets and variables

BISMO involves a somewhat complex hardware-software build, which is currently managed using the top-level `Makefile`. Below are some of the useful variables and targets.

### Variables
| Variable        | Description  | Default value |
| ------------- |:-------------:|-------------:|
| `PLATFORM` | Select the platform to use, see below | `PYNQU96` |
| `${PLATFORM}_URI` | URI to be used as rsync deployment target | none |
| `FREQ_MHZ` | Target clock frequency for synthesis | 200.0 |
| `M` | LHS parallelism for overlay; see Dm in BISMO paper | 2 |
| `K` | Inner product parallelism for overlay; see Dk in BISMO paper | 64 |
| `N` | RHS parallelism for overlay; see Dn in BISMO paper | 2 |
| `LMEM` | Number of entries in LHS memory; see Bm in BISMO paper | 1024 |
| `RMEM` | Number of entries in RHS memory; see Bn in BISMO paper | 1024 |
| `BUILD_DIR` | Build directory for current BISMO instance | build/$(OVERLAY_CFG)/$(PLATFORM) |

### Make targets

| Target        | Description  |
| ------------- |:-------------:|
| `all` | Create a deployment package including hardware, software and scripts |
| `resmodel` | Estimate FPGA resources for current overlay configuration |
| `report` | Print the resource usage for key FPGA resources after synthesis |
| `rsync` | Copy generated deployment package to target URI using rsync |

Note that targets can be platform specific. For instance, `report` does not
exist for the `VerilatedTester` platform.


## Platforms

BISMO can be deployed on a variety of boards using Xilinx FPGAs using the
same hardware and software.
You can specify which platform to use by using the `PLATFORM` environment
variable e.g. `PLATFORM=PYNQU96 make all`.

### Supported platforms

| PLATFORM        | Board       | Remarks  | Largest configuration  |
| ------------- |:-------------:| -----:| -----:|
| `VerilatedTester`      | none (emulated) | Emulates 64-bit fixed-latency DRAM | depends on your host system |
| `PYNQZ1`      | Xilinx PYNQ-Z1 | Uses 64-bit AXI HP0 port | 8x256x8@200 MHz: 6.5 binary TOPS |
| `PYNQU96`      | Avnet Ultra96 |  Uses 64-bit AXI HP0 port | 10x256x10@300 MHz: 15 binary TOPS |
| `PYNQU96CC` | Avnet Ultra96 | (experimental) Support for coherent memory using 64-bit HPC0 port | 10x256x10@300 MHz: 15 binary TOPS |

### Using VerilatedTester for emulation

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

For emulation support on mac, try the following:
1. `brew tap caskroom/versions`
2. `brew cask install java8`
3. `brew install sbt@1`

### Adding support for a new platform

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
