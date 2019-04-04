#!/bin/bash
VERILATOR_SRC_DIR="/usr/share/verilator/include"

g++ -std=c++11 -march=native -O0 -I$VERILATOR_SRC_DIR -Iverilog/verilated -I./hls_include -I./driver -I./test -fPIC inflib/*.cpp driver/*.cpp verilog/verilated/*.cpp -shared -o libbismo_inference.so
