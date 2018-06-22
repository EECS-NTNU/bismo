# HW-SW Cosimulation Tests

This folder contains the C++ source files that constitute the software part of
hardware-software cosimulation tests for several BISMO components.
This uses the [fpga-tidbits PlatformWrapper virtual
platform](https://github.com/maltanar/fpga-tidbits/wiki/platformwrapper) as
the infrastructure.
For each component, the corresponding hardware part is defined under
`src/main/scala/cosim`.
The hardware part for BISMO stages may include some additional I/O compared to
the regular interfaces in BISMO to make it possible to alter and observe the
hardware state (such as direct CPU access to the result memory).

Each source file `EmuTest<Module>.scala` contains a hardware-software
cosimulation test for `<Module>`, and can be
invoked as a make target from the root folder of the repository.
For instance, run `make EmuTestExecStage` to run the test for `ExecStage`.
