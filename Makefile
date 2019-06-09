# Copyright (c) 2018 Norwegian University of Science and Technology (NTNU)
# Copyright (c) 2019 Xilinx
#
# BSD v3 License
#
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# * Redistributions of source code must retain the above copyright notice, this
#   list of conditions and the following disclaimer.
#
# * Redistributions in binary form must reproduce the above copyright notice,
#   this list of conditions and the following disclaimer in the documentation
#   and/or other materials provided with the distribution.
#
# * Neither the name of BISMO nor the names of its
#   contributors may be used to endorse or promote products derived from
#   this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

# target frequency for Vivado FPGA synthesis
FREQ_MHZ ?= 200.0
# controls whether Vivado will run in command-line or GUI mode
VIVADO_MODE ?= batch # or gui
# which C++ compiler to use
CC = g++
# scp/rsync target to copy files to board
PLATFORM ?= PYNQU96
URI = $($(PLATFORM)_URI)
# overlay dims
M ?= 2
K ?= 64
N ?= 2
LMEM ?= 1024
RMEM ?= 1024
O ?= 64
F ?= true
OVERLAY_CFG = $(M)x$(K)x$(N)

# other project settings
SBT ?= sbt
SBT_FLAGS ?= -Dsbt.log.noformat=true
# internal build dirs and names for the Makefile
TOP ?= $(shell dirname $(realpath $(filter %Makefile, $(MAKEFILE_LIST))))
TIDBITS_ROOT ?= $(TOP)/fpga-tidbits
TIDBITS_REGDRV_ROOT ?= $(TIDBITS_ROOT)/src/main/resources/cpp/platform-wrapper-regdriver
BUILD_DIR ?= $(TOP)/build/$(OVERLAY_CFG)/$(PLATFORM)
BUILD_DIR_CLEAN ?= $(TOP)/build/$(OVERLAY_CFG)
BUILD_DIR_CHARACTERIZE := $(BUILD_DIR)/characterize
BUILD_DIR_DEPLOY := $(BUILD_DIR)/deploy
BUILD_DIR_VERILOG := $(BUILD_DIR)/hw/verilog
BUILD_DIR_HWDRV := $(BUILD_DIR)/hw/driver
BUILD_DIR_RTLIB := $(BUILD_DIR)/rtlib
VERILOG_SRC_DIR := $(TIDBITS_ROOT)/src/main/resources/verilog
VHDL_SRC_DIR := $(TOP)/src/main/vhdl
APP_SRC_DIR := $(TOP)/src/main/resources/cpp/app
RTLIB_SRC_DIR := $(TOP)/src/main/resources/cpp/lib
HLS_SRC_DIR := $(TOP)/src/main/resources/hls
HLSTEST_SRC_DIR := $(TOP)/src/main/resources/hls/test
VIVADO_IN_PATH := $(shell command -v vivado 2> /dev/null)
CPPTEST_SRC_DIR := $(TOP)/src/test/cosim
HW_VERILOG := $(BUILD_DIR_VERILOG)/$(PLATFORM)Wrapper.v
HW_TO_SYNTH ?= $(HW_VERILOG) $(BUILD_DIR_VERILOG)/ExecInstrGen.v
HW_SW_DRIVER ?= BitSerialMatMulAccel.hpp
PLATFORM_SCRIPT_DIR := $(TOP)/src/main/script/$(PLATFORM)/target
VIVADOHLS_ROOT ?= $(shell dirname $(shell which vivado_hls))/..
HLS_SIM_INCL := $(VIVADOHLS_ROOT)/include
DEBUG_CHISEL ?= 0
CC_FLAG =
# platform-specific Makefile include for bitfile synthesis
include platforms/$(PLATFORM).mk

# note that all targets are phony targets, no proper dependency tracking
.PHONY: hw_verilog hw_driver hw_vivadoproj bitfile hw sw all rsync test
.PHONY: resmodel characterize check_vivado pretty p2saccel benchmark

# note that all targets are phony targets, no proper dependency tracking
.PHONY: hw_verilog emulib hw_driver hw_vivadoproj bitfile hw sw all rsync test characterize check_vivado emu emu_cfg

# run Scala/Chisel tests
Test%:
	$(SBT) $(SBT_FLAGS) "test-only $@"

# run HLS (non-synthesis, software only) tests
HLSTest%:
	mkdir -p $(BUILD_DIR)/$@; \
	cd $(BUILD_DIR)/$@; \
	cp $(HLSTEST_SRC_DIR)/$@.cpp .; \
	cp $(HLSTEST_SRC_DIR)/$*_TemplateDefs.hpp .; \
	cp $(HLS_SRC_DIR)/$*.cpp .; \
	cp $(RTLIB_SRC_DIR)/BISMOInstruction.cpp .; \
	g++ -std=c++11 -I$(RTLIB_SRC_DIR) -I$(HLS_SIM_INCL) *.cpp -o $@; \
	./$@

# run resource/Fmax characterization
Characterize%:
	mkdir -p $(BUILD_DIR)/$@; cp $(VERILOG_SRC_DIR)/*.v $(BUILD_DIR)/$@; cp $(VHDL_SRC_DIR)/*.vhd $(BUILD_DIR)/$@;$(SBT) $(SBT_FLAGS) "runMain bismo.CharacterizeMain $@ $(BUILD_DIR)/$@ $(PLATFORM)"

# generate register driver for the Chisel accelerator
hw_driver: $(BUILD_DIR_HWDRV)/BitSerialMatMulAccel.hpp

$(BUILD_DIR_HWDRV)/BitSerialMatMulAccel.hpp:
	mkdir -p "$(BUILD_DIR_HWDRV)"
	$(SBT) $(SBT_FLAGS) "runMain bismo.DriverMain $(PLATFORM) $(BUILD_DIR_HWDRV) $(TIDBITS_REGDRV_ROOT)"

# generate Verilog for the Chisel accelerator
hw_verilog: $(HW_VERILOG)

$(HW_VERILOG):
	$(SBT) $(SBT_FLAGS) "runMain bismo.ChiselMain $(PLATFORM) $(BUILD_DIR_VERILOG) $(M) $(K) $(N) $(LMEM) $(RMEM)"

hls: $(BUILD_DIR_VERILOG)/ExecInstrGen.v

$(BUILD_DIR_VERILOG)/ExecInstrGen.v:
	mkdir -p $(BUILD_DIR_VERILOG); \
	$(SBT) $(SBT_FLAGS) "runMain bismo.HLSMain $(PLATFORM) $(FREQ_MHZ) $(BUILD_DIR_VERILOG) $(M) $(K) $(N) $(LMEM) $(RMEM)"

resmodel:
	$(SBT) $(SBT_FLAGS) "runMain bismo.ResModelMain $(PLATFORM) $(BUILD_DIR_VERILOG) $(M) $(K) $(N) $(LMEM) $(RMEM) $(FREQ_MHZ)"

# copy scripts to the deployment folder
script:
	mkdir -p $(BUILD_DIR_DEPLOY); cp -f $(PLATFORM_SCRIPT_DIR)/* $(BUILD_DIR_DEPLOY)/

# get everything ready to copy onto the platform and create a deployment folder
all: hw sw script

# use rsync to synchronize contents of the deployment folder onto the platform
rsync:
	rsync -avz $(BUILD_DIR_DEPLOY) $(URI)
benchmark:
	rsync $(URI)/deploy/benchmark* $(TOP)/

# remove everything that is built
clean:
	rm -rf $(BUILD_DIR_CLEAN)
# remove everthing for that platform
cleanplat:
	rm -rf $(BUILD_DIR)
# download scalariform
scalariform.jar:
	wget https://github.com/scala-ide/scalariform/releases/download/0.2.6/scalariform.jar

# format source code with scalariform
pretty: scalariform.jar
	java -jar scalariform.jar --recurse src/main/scala
