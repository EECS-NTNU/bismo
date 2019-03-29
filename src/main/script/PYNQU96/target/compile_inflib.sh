#!/bin/bash
g++ -std=c++11 -march=native -O3 -I./hls_include -I./driver -I./test -fPIC inflib/*.cpp -lcma -shared -o libbismo_inference.so
