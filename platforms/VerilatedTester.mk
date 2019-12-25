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

VERILATOR_SRC_DIR ?= /usr/share/verilator/include

$(BUILD_DIR_DEPLOY)/verilog/verilated: $(HW_TO_SYNTH)
	mkdir -p $(BUILD_DIR_DEPLOY)/verilog; \
	cp -rf $(BUILD_DIR_VERILOG)/* $(BUILD_DIR_DEPLOY)/verilog; \
	cp -rf $(VERILOG_SRC_DIR)/* $(BUILD_DIR_DEPLOY)/verilog; \
	cd $(BUILD_DIR_DEPLOY)/verilog; \
	verilator -Iother-verilog --cc TesterWrapper.v -Wno-assignin -Wno-fatal -Wno-lint -Wno-style -Wno-COMBDLY -Wno-STMTDLY --Mdir verilated --trace; \
	cp -rf $(VERILATOR_SRC_DIR)/verilated.cpp $(BUILD_DIR_DEPLOY)/verilog/verilated; \
	cp -rf $(VERILATOR_SRC_DIR)/verilated_vcd_c.cpp $(BUILD_DIR_DEPLOY)/verilog/verilated;

hw: $(BUILD_DIR_DEPLOY)/verilog/verilated

# copy all user sources and driver sources to the deployment folder
sw: $(BUILD_DIR_HWDRV)/$(HW_SW_DRIVER)
	mkdir -p $(BUILD_DIR_DEPLOY); \
	mkdir -p $(BUILD_DIR_DEPLOY)/driver; \
	mkdir -p $(BUILD_DIR_DEPLOY)/test; \
	mkdir -p $(BUILD_DIR_DEPLOY)/rtlib; \
	mkdir -p $(BUILD_DIR_DEPLOY)/hls_include; \
	cp -rf $(BUILD_DIR_HWDRV)/* $(BUILD_DIR_DEPLOY)/driver/; \
	cp -rf $(APP_SRC_DIR)/* $(BUILD_DIR_DEPLOY)/test/;
	cp -rf $(RTLIB_SRC_DIR)/* $(BUILD_DIR_DEPLOY)/rtlib; \
	cp -rf $(HLS_SIM_INCL)/* $(BUILD_DIR_DEPLOY)/hls_include;

emu: rtlib_emu
	cd $(BUILD_DIR_DEPLOY); \
	sh compile_testapp.sh; \
	LD_LIBRARY_PATH=$(BUILD_DIR_DEPLOY):$(LD_LIBRARY_PATH) ./testapp t;

$(BUILD_DIR_DEPLOY)/libbismo_rt.so: hw sw script
	cd $(BUILD_DIR_DEPLOY); \
	sh compile_rtlib.sh;

rtlib_emu: $(BUILD_DIR_DEPLOY)/libbismo_rt.so

# hw-sw cosimulation tests with extra HLS dependencies
EmuTest%:
	mkdir -p $(BUILD_DIR)/$@; \
	$(SBT) $(SBT_FLAGS) "runMain bismo.EmuLibMain $@ $(BUILD_DIR)/$@ verilator $(DEBUG_CHISEL)"; \
	cp -rf $(CPPTEST_SRC_DIR)/$@.cpp $(BUILD_DIR)/$@; \
	ln -s $(RTLIB_SRC_DIR)/BISMOInstruction.* $(BUILD_DIR)/$@/; \
	cd $(BUILD_DIR)/$@; sh verilator-build.sh -I$(HLS_SIM_INCL); ./VerilatedTesterWrapper

#BUILD_DIR_EMU := $(BUILD_DIR)/emu
#BUILD_DIR_EMULIB_CPP := $(BUILD_DIR)/hw/cpp_emulib
#
#
#EmuTestExecInstrGen:
#	mkdir -p $(BUILD_DIR)/$@;
#	$(SBT) $(SBT_FLAGS) "runMain bismo.EmuLibMain $@ $(BUILD_DIR)/$@ verilator $(DEBUG_CHISEL)";
#	cp -rf $(CPPTEST_SRC_DIR)/$@.cpp $(BUILD_DIR)/$@;
#	ln -s $(RTLIB_SRC_DIR)/*.hpp $(BUILD_DIR)/$@;
#	ln -s $(APP_SRC_DIR)/gemmbitserial $(BUILD_DIR)/$@;
#	cd $(BUILD_DIR)/$@; sh verilator-build.sh -I$(HLS_SIM_INCL); ./VerilatedTesterWrapper
#
#EmuTestFetchInstrGen:
#	mkdir -p $(BUILD_DIR)/$@;
#	$(SBT) $(SBT_FLAGS) "runMain bismo.EmuLibMain $@ $(BUILD_DIR)/$@ verilator $(DEBUG_CHISEL)";
#	cp -rf $(CPPTEST_SRC_DIR)/$@.cpp $(BUILD_DIR)/$@;
#	ln -s $(RTLIB_SRC_DIR)/*.hpp $(BUILD_DIR)/$@;
#	ln -s $(APP_SRC_DIR)/gemmbitserial $(BUILD_DIR)/$@;
#	cd $(BUILD_DIR)/$@; sh verilator-build.sh -I$(HLS_SIM_INCL); ./VerilatedTesterWrapper
#
#
## run hardware-software cosimulation tests
#EmuTest%:
#	mkdir -p $(BUILD_DIR)/$@; $(SBT) $(SBT_FLAGS) "runMain bismo.EmuLibMain $@ $(BUILD_DIR)/$@ cpp $(DEBUG_CHISEL)"; cp -r $(CPPTEST_SRC_DIR)/$@.cpp $(BUILD_DIR)/$@; cp -r $(APP_SRC_DIR)/gemmbitserial $(BUILD_DIR)/$@; cd $(BUILD_DIR)/$@; g++ -std=c++11 *.cpp driver.a -o $@; ./$@
#
## run hardware-software cosimulation tests (in debug mode with waveform dump)
#DebugEmuTest%:
#	mkdir -p $(BUILD_DIR)/$@; $(SBT) $(SBT_FLAGS) "runMain bismo.EmuLibMain EmuTest$* $(BUILD_DIR)/$@ cpp 1"; cp -r $(CPPTEST_SRC_DIR)/EmuTest$*.cpp $(BUILD_DIR)/$@; cp -r $(APP_SRC_DIR)/gemmbitserial $(BUILD_DIR)/$@; cd $(BUILD_DIR)/$@; g++ -std=c++11 -DDEBUG *.cpp driver.a -o $@; ./$@
#
## generate emulator executable including software sources
#emu: $(BUILD_DIR_EMU)/verilator-build.sh
#	cp -rf $(APP_SRC_DIR)/* $(BUILD_DIR_EMU)/;
#	cp -rf $(RTLIB_SRC_DIR)/* $(BUILD_DIR_EMU)/; \
#	cd $(BUILD_DIR_EMU); sh verilator-build.sh -I$(HLS_SIM_INCL); mv VerilatedTesterWrapper emu; ./emu t
#
## generate cycle-accurate C++ emulator for the whole system via Verilator
#$(BUILD_DIR_EMU)/verilator-build.sh: $(BUILD_DIR_VERILOG)/ExecInstrGen.v
#	mkdir -p $(BUILD_DIR_EMU); \
#	$(SBT) $(SBT_FLAGS) "runMain bismo.EmuLibMain main $(BUILD_DIR_EMU) verilator $(DEBUG_CHISEL)"; \
#	cp -rf $(BUILD_DIR_VERILOG)/* $(BUILD_DIR_EMU)/
#
#
#
## generate dynamic lib for inference, emulated hardware
#rtlib_emu: $(BUILD_DIR_EMU)/verilator-build.sh
#	rm -rf $(BUILD_DIR_RTLIB); \
#	mkdir -p $(BUILD_DIR_RTLIB); \
#	cp -rf $(BUILD_DIR_EMU)/* $(BUILD_DIR_RTLIB)/; \
#	cp -rf $(RTLIB_SRC_DIR)/* $(BUILD_DIR_RTLIB)/; \
#	cd $(BUILD_DIR_RTLIB); \
#	verilator -Iother-verilog --cc TesterWrapper.v -Wno-assignin -Wno-fatal -Wno-lint -Wno-style -Wno-COMBDLY -Wno-STMTDLY --Mdir verilated --trace; \
#	cp -rf $(VERILATOR_SRC_DIR)/verilated.cpp .; \
#	cp -rf $(VERILATOR_SRC_DIR)/verilated_vcd_c.cpp .; \
#	g++ -std=c++11 -I$(HLS_SIM_INCL) -I$(BUILD_DIR_EMU) -Iverilated -I$(VERILATOR_SRC_DIR) -I$(APP_SRC_DIR) -fPIC verilated/*.cpp *.cpp -shared -o $(BUILD_DIR_RTLIB)/libbismo_rt.so

#
