# Chisel Tests

This folder contains Chisel tests for a number of BISMO components.
Each source file `Test<Module>.scala` contains a test for `<Module>`, and can be
invoked as a make target from the root folder of the repository.
For instance, run `make TestPopCountUnit` to run the test for `PopCountUnit`.

These are relatively simple tests running inside Chisel only, covering the
basic modules.
Most of them will complete in seconds, and the rest will take several minutes.
The more complex modules are tested in emulation with a small C++ application.
Consult `src/test/cpp/README.md` for more information on those.
