#!/bin/bash
g++ -std=c++11 -march=native -O3 -I./include -I./driver -I./app -fPIC *.cpp -lcma -shared -o libbismo_inference.so
