# Hardware
BISMO is implemented in [Chisel 2](https://chisel.eecs.berkeley.edu) using
components from the [fpga-tidbits](https://github.com/maltanar/fpga-tidbits/)
framework, and targets Xilinx FPGAs.
The Chisel source code can be found under `src/main/scala`.
Currently there is no separate documentation available for the hardware design,
the best sources of information are the paper and the comments in the code.

## Overlay Configuration
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
`makeParamSpace` functions under `CharacterizeMain`.
