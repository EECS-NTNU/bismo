# Copyright (c) 2018 Norwegian University of Science and Technology (NTNU)
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
# * Neither the name of [project] nor the names of its
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
FREQ_MHZ ?= 150.0
# controls whether Vivado will run in command-line or GUI mode
VIVADO_MODE ?= batch # or gui
# which C++ compiler to use
CC = g++
# scp/rsync target to copy files to board
PLATFORM ?= PYNQZ1
URI = $($(PLATFORM)_URI)
# overlay dims
M ?= 8
K ?= 256
N ?= 8
O ?= 64
F ?= true
OVERLAY_CFG = $(M)x$(K)x$(N)

# other project settings
SBT ?= sbt
SBT_FLAGS ?= -Dsbt.log.noformat=true
# internal build dirs and names for the Makefile
TOP ?= $(shell readlink -f .)
TIDBITS_ROOT ?= $(TOP)/fpga-tidbits
TIDBITS_REGDRV_ROOT ?= $(TIDBITS_ROOT)/src/main/resources/cpp/platform-wrapper-regdriver
export OHMYXILINX := $(TOP)/oh-my-xilinx
export PATH := $(PATH):$(OHMYXILINX)
BUILD_DIR ?= $(TOP)/build/$(OVERLAY_CFG)/$(PLATFORM)
BUILD_DIR_CLEAN ?= $(TOP)/build/$(OVERLAY_CFG)
BUILD_DIR_CHARACTERIZE := $(BUILD_DIR)/characterize
BUILD_DIR_DEPLOY := $(BUILD_DIR)/deploy
BUILD_DIR_VERILOG := $(BUILD_DIR)/hw/verilog
BUILD_DIR_EMU := $(BUILD_DIR)/emu
BUILD_DIR_HWDRV := $(BUILD_DIR)/hw/driver
BUILD_DIR_EMULIB_CPP := $(BUILD_DIR)/hw/cpp_emulib
BUILD_DIR_INFLIB := $(BUILD_DIR)/inflib
VERILOG_SRC_DIR := $(TOP)/src/main/verilog
VHDL_SRC_DIR := $(TOP)/src/main/vhdl
APP_SRC_DIR := $(TOP)/src/main/resources/cpp/app
INFLIB_SRC_DIR := $(TOP)/src/main/resources/cpp/lib
VIVADO_IN_PATH := $(shell command -v vivado 2> /dev/null)
ZSH_IN_PATH := $(shell command -v zsh 2> /dev/null)
CPPTEST_SRC_DIR := $(TOP)/src/test/cosim
HW_VERILOG := $(BUILD_DIR_VERILOG)/$(PLATFORM)Wrapper.v
HW_TO_SYNTH ?= $(HW_VERILOG)
HW_SW_DRIVER ?= BitSerialMatMulAccel.hpp
PLATFORM_SCRIPT_DIR := $(TOP)/src/main/script/$(PLATFORM)/target
VIVADOHLS_ROOT ?= $(shell dirname $(shell which vivado_hls))/..
HLS_SIM_INCL := $(VIVADOHLS_ROOT)/include
DEBUG_CHISEL ?= 0
CC_FLAG =
VERILATOR_SRC_DIR = /usr/share/verilator/include
# platform-specific Makefile include for bitfile synthesis
include platforms/$(PLATFORM).mk

# note that all targets are phony targets, no proper dependency tracking
.PHONY: hw_verilog emulib hw_driver hw_vivadoproj bitfile hw sw all rsync test
.PHONY: resmodel characterize check_vivado pretty p2saccel benchmark

check_vivado:
ifndef VIVADO_IN_PATH
    $(error "vivado not found in path")
endif

check_zsh:
ifndef ZSH_IN_PATH
    $(error "zsh not in path; needed by oh-my-xilinx for characterization")
endif

# hw-sw cosimulation tests with extra HLS dependencies
EmuTestVerifyHLSInstrEncoding:
	mkdir -p $(BUILD_DIR)/$@;
	$(SBT) $(SBT_FLAGS) "runMain bismo.EmuLibMain $@ $(BUILD_DIR)/$@ verilator";
	cp -r $(CPPTEST_SRC_DIR)/$@.cpp $(BUILD_DIR)/$@;
	ln -s $(APP_SRC_DIR)/*.hpp $(BUILD_DIR)/$@;
	ln -s $(APP_SRC_DIR)/gemmbitserial $(BUILD_DIR)/$@;
	cd $(BUILD_DIR)/$@; sh verilator-build.sh -I$(HLS_SIM_INCL); ./VerilatedTesterWrapper

EmuTestExecInstrGen:
	mkdir -p $(BUILD_DIR)/$@;
	$(SBT) $(SBT_FLAGS) "runMain bismo.EmuLibMain $@ $(BUILD_DIR)/$@ verilator";
	cp -r $(CPPTEST_SRC_DIR)/$@.cpp $(BUILD_DIR)/$@;
	ln -s $(APP_SRC_DIR)/*.hpp $(BUILD_DIR)/$@;
	ln -s $(APP_SRC_DIR)/gemmbitserial $(BUILD_DIR)/$@;
	cd $(BUILD_DIR)/$@; sh verilator-build.sh -I$(HLS_SIM_INCL); ./VerilatedTesterWrapper

EmuTestFetchInstrGen:
	mkdir -p $(BUILD_DIR)/$@;
	$(SBT) $(SBT_FLAGS) "runMain bismo.EmuLibMain $@ $(BUILD_DIR)/$@ verilator";
	cp -r $(CPPTEST_SRC_DIR)/$@.cpp $(BUILD_DIR)/$@;
	ln -s $(APP_SRC_DIR)/*.hpp $(BUILD_DIR)/$@;
	ln -s $(APP_SRC_DIR)/gemmbitserial $(BUILD_DIR)/$@;
	cd $(BUILD_DIR)/$@; sh verilator-build.sh -I$(HLS_SIM_INCL); ./VerilatedTesterWrapper

# run Scala/Chisel tests
Test%:
	$(SBT) $(SBT_FLAGS) "test-only $@"

# run hardware-software cosimulation tests (in debug mode with waveform dump)
DebugEmuTest%:
	mkdir -p $(BUILD_DIR)/$@; $(SBT) $(SBT_FLAGS) "runMain bismo.EmuLibMain EmuTest$* $(BUILD_DIR)/$@ cpp 1"; cp -r $(CPPTEST_SRC_DIR)/EmuTest$*.cpp $(BUILD_DIR)/$@; cp -r $(APP_SRC_DIR)/gemmbitserial $(BUILD_DIR)/$@; cd $(BUILD_DIR)/$@; g++ -std=c++11 -DDEBUG *.cpp driver.a -o $@; ./$@

# run hardware-software cosimulation tests
EmuTest%:
	mkdir -p $(BUILD_DIR)/$@; $(SBT) $(SBT_FLAGS) "runMain bismo.EmuLibMain $@ $(BUILD_DIR)/$@ cpp $(DEBUG_CHISEL)"; cp -r $(CPPTEST_SRC_DIR)/$@.cpp $(BUILD_DIR)/$@; cp -r $(APP_SRC_DIR)/gemmbitserial $(BUILD_DIR)/$@; cd $(BUILD_DIR)/$@; g++ -std=c++11 *.cpp driver.a -o $@; ./$@

# generate cycle-accurate C++ emulator for the whole system via Verilator
$(BUILD_DIR_EMU)/verilator-build.sh:
	mkdir -p $(BUILD_DIR_EMU); $(SBT) $(SBT_FLAGS) "runMain bismo.EmuLibMain main $(BUILD_DIR_EMU) verilator $(DEBUG_CHISEL)"

# generate emulator executable including software sources
emu: $(BUILD_DIR_EMU)/verilator-build.sh
	cp -r $(APP_SRC_DIR)/* $(BUILD_DIR_EMU)/;
	cd $(BUILD_DIR_EMU); sh verilator-build.sh -I$(HLS_SIM_INCL); mv VerilatedTesterWrapper emu; ./emu

# generate dynamic lib for inference, emulated hardware
inflib_emu: $(BUILD_DIR_EMU)/verilator-build.sh
	mkdir -p $(BUILD_DIR_INFLIB); \
	cp -r $(INFLIB_SRC_DIR)/* $(BUILD_DIR_INFLIB)/; \
	cp $(BUILD_DIR_EMU)/* $(BUILD_DIR_INFLIB)/; \
	cd $(BUILD_DIR_INFLIB); \
	verilator -Iother-verilog --cc TesterWrapper.v -Wno-assignin -Wno-fatal -Wno-lint -Wno-style -Wno-COMBDLY -Wno-STMTDLY --Mdir verilated --trace; \
	cp -f $(VERILATOR_SRC_DIR)/verilated.cpp .; \
	cp -f $(VERILATOR_SRC_DIR)/verilated_vcd_c.cpp .; \
	g++ -std=c++11 -I$(HLS_SIM_INCL) -I$(BUILD_DIR_EMU) -Iverilated -I$(VERILATOR_SRC_DIR) -I$(APP_SRC_DIR) -fPIC verilated/*.cpp *.cpp -shared -o $(BUILD_DIR_INFLIB)/libbismo_inference.so

inflib: hw_driver
	mkdir -p $(BUILD_DIR_INFLIB); \
	cp -r $(INFLIB_SRC_DIR)/* $(BUILD_DIR_INFLIB)/; \
	cp $(BUILD_DIR_HWDRV)/*.cpp $(BUILD_DIR_INFLIB)/; \
	cd $(BUILD_DIR_INFLIB); \
	g++ -std=c++11 -I$(BUILD_DIR_HWDRV) -I$(APP_SRC_DIR) -fPIC *.cpp -shared -o $(BUILD_DIR_INFLIB)/libbismo_inference.so

# run resource/Fmax characterization
Characterize%:
	mkdir -p $(BUILD_DIR)/$@; cp $(VERILOG_SRC_DIR)/*.v $(BUILD_DIR)/$@; cp $(VHDL_SRC_DIR)/*.vhd $(BUILD_DIR)/$@;$(SBT) $(SBT_FLAGS) "runMain bismo.CharacterizeMain $@ $(BUILD_DIR)/$@ $(PLATFORM)"

# generate register driver for the Chisel accelerator
hw_driver: $(BUILD_DIR_HWDRV)/BitSerialMatMulAccel.hpp

$(BUILD_DIR_HWDRV)/BitSerialMatMulAccel.hpp:
	mkdir -p "$(BUILD_DIR_HWDRV)"; $(SBT) $(SBT_FLAGS) "runMain bismo.DriverMain $(PLATFORM) $(BUILD_DIR_HWDRV) $(TIDBITS_REGDRV_ROOT)"
#driver for the p2s
$(BUILD_DIR_HWDRV)/EmuTestP2SAccel.hpp:
	mkdir -p "$(BUILD_DIR_HWDRV)";$(SBT) $(SBT_FLAGS) "runMain bismo.P2SDriverMain $(PLATFORM) $(BUILD_DIR_HWDRV) $(TIDBITS_REGDRV_ROOT)"

# generate Verilog for the Chisel accelerator
hw_verilog: $(HW_VERILOG)

$(HW_VERILOG):
	$(SBT) $(SBT_FLAGS) "runMain bismo.ChiselMain $(PLATFORM) $(BUILD_DIR_VERILOG) $M $K $N"

resmodel:
	$(SBT) $(SBT_FLAGS) "runMain bismo.ResModelMain $(PLATFORM) $(BUILD_DIR_VERILOG) $M $K $N"
#generate for p2s
p2s:
	$(SBT) $(SBT_FLAGS) "runMain bismo.P2SMain $(PLATFORM) $(BUILD_DIR_VERILOG) $M $N $O $F"
#generate the drivers and copy into the deploy
p2ssw: $(BUILD_DIR_HWDRV)/EmuTestP2SAccel.hpp
	mkdir -p $(BUILD_DIR_DEPLOY); cp $(BUILD_DIR_HWDRV)/* $(BUILD_DIR_DEPLOY)/; cp -r $(CPPTEST_SRC_DIR)/EmuTestP2SAccel.cpp $(BUILD_DIR_DEPLOY)/; cp -r $(APP_SRC_DIR)/gemmbitserial $(BUILD_DIR_DEPLOY)/

# copy scripts to the deployment folder
script:
	cp $(PLATFORM_SCRIPT_DIR)/* $(BUILD_DIR_DEPLOY)/

# get everything ready to copy onto the platform and create a deployment folder
all: hw sw script

p2saccel:hw p2ssw script

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
