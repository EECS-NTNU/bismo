# Hardware
BISMO is implemented mostly in [Chisel 2](https://chisel.eecs.berkeley.edu) using
components from the [fpga-tidbits](https://github.com/maltanar/fpga-tidbits/)
framework, and targets Xilinx FPGAs.
It also uses several Verilog, VHDL and Vivado HLS-generated components.
The Chisel source code can be found under `src/main/scala`.
Currently there is no separate documentation available for the hardware design,
the best sources of information are the
[BISMO paper](https://arxiv.org/pdf/1901.00370.pdf) and the comments in the code.

## Mixed-Language design

* Chisel 2 is used for most of the design, including instantiating and
combining together the components written in other HDLs. We accomplish this by
defining Chisel `BlackBox` modules that have the same signal-level interface
as the components built in other languages.

* The FPGA-optimized compressor generator is written in VHDL. When targeting
the `VerilatedTester` platform, this code is not used; simpler compressors
written in Chisel are used instead. This is because Verilator does not accept
VHDL inputs.

* Verilog is used for dual-port BRAMs and FPGA-optimized queues, as provided
by fpga-tidbits.

* Vivado HLS is used for the instruction generators.

## Instruction Generators

<center> <img src="img/bismo-instruction-generators.png"></center>

Processing large matrices with BISMO requires many instructions, especially
for the execute stage. These instructions would consume significant memory
bandwidth and storage if explicitly provided to the accelerator. To overcome
this, we use *instruction generators* in BISMO. These are small pieces of
hardware that take in the description of the large matrix multiply operation,
and generate the corresponding instruction stream for each stage.

**How are the instruction generators implemented?**
The instruction generators are implemented in Vivado HLS since tiled matrix
multiplication corresponds to a well-structured nested loop over
different tiles and bit positions. You can find the HLS source code under
`src/main/resources/hls` and the corresponding Chisel `BlackBox` modules under
`src/main/scala` (using the same names).

**What if I want to use my own instructions?**
The instruction generators can be bypassed
using a multiplexer if desired, enabling the user to directly feed BISMO with
their own instructions. Use the
`BitSerialMatMulAccel::useDirectInstructionFeed()` function to configure the
instruction mux, then feed your instructions using
`BitSerialMatMulAccel::pushInstruction()`.

## Overlay Configuration
BISMO is parametrized and can be instantiated in different sizes to generate a
higher-performance overlay using more FPGA resources.

* *Directly in the source code:* The actual instantiaton is carried out in
`ChiselMain.main` defined in `src/main/scala/Main.scala` which uses the
`BitSerialMatMulParams` class defined in `src/main/scala/BISMO.scala` to specify
the overlay configuration.
You can specify the overlay dimensions directly in `ChiselMain.main`.

* *As environment variables (limited):* For quick experimentation, some of the
overlay dimensions are sourced from environment variables when `make` is called.
See under Makefile variables [here](platforms.md) for more details.

## Resource Characterization Flow
You can run the characterization flow to see how the FPGA resource usage
varies with configuration for different BISMO components.
To run the characterization, simply use the name defined under
`CharacterizeMain.main` as a `make` target, e.g. `make CharacterizePC` will
run the characterization for the PopCountUnit.
This will produce a file `CharacterizePC.log` with the results.
Completing the characterization will take some time depending on the range of
parameters.
To change the range of parameters for characterization, see the
`makeParamSpace` functions under `CharacterizeMain` in `src/main/scala/MainCharacterize.scala`.
