#!/bin/bash
g++ -std=c++11 -I./include -I./driver -I./app -fPIC *.cpp -lcma -shared -o libbismo_inference.so
