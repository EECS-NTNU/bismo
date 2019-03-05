#!/bin/bash
g++ -std=c++11 -DGEMMBITSERIAL_USE_ARM_NEON -I./include -I./driver -I./app -fPIC *.cpp -lcma -shared -o libbismo_inference.so
