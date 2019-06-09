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

BITFILE_PRJNAME := bitfile_synth
BITFILE_PRJDIR := $(BUILD_DIR)/bitfile_synth
GEN_BITFILE_PATH := $(BITFILE_PRJDIR)/$(BITFILE_PRJNAME).runs/impl_1/procsys_wrapper.bit
VIVADO_PROJ_SCRIPT := $(TOP)/src/main/script/$(PLATFORM)/host/make-vivado-project.tcl
VIVADO_SYNTH_SCRIPT := $(TOP)/src/main/script/$(PLATFORM)/host/synth-vivado-project.tcl

# create a new Vivado project
hw_vivadoproj: $(BITFILE_PRJDIR)/bitfile_synth.xpr

$(BITFILE_PRJDIR)/bitfile_synth.xpr: $(HW_TO_SYNTH)
	vivado -mode $(VIVADO_MODE) -source $(VIVADO_PROJ_SCRIPT) -tclargs $(TOP) $(BUILD_DIR_VERILOG) $(BITFILE_PRJNAME) $(BITFILE_PRJDIR) $(FREQ_MHZ)

# launch Vivado in GUI mode with created project
launch_vivado_gui: $(BITFILE_PRJDIR)/bitfile_synth.xpr
	vivado -mode gui $(BITFILE_PRJDIR)/$(BITFILE_PRJNAME).xpr

# run bitfile synthesis for the Vivado project
bitfile: $(GEN_BITFILE_PATH)

$(GEN_BITFILE_PATH): $(BITFILE_PRJDIR)/bitfile_synth.xpr
	vivado -mode $(VIVADO_MODE) -source $(VIVADO_SYNTH_SCRIPT) -tclargs $(BITFILE_PRJDIR)/$(BITFILE_PRJNAME).xpr

# copy bitfile to the deployment folder, make an empty tcl script for bitfile loader
hw: $(GEN_BITFILE_PATH)
	mkdir -p $(BUILD_DIR_DEPLOY); cp $(GEN_BITFILE_PATH) $(BUILD_DIR_DEPLOY)/bismo.bit; cp $(BITFILE_PRJDIR)/bismo.tcl $(BUILD_DIR_DEPLOY)/bismo.tcl

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

report: $(GEN_BITFILE_PATH)
	cat $(BITFILE_PRJDIR)/$(BITFILE_PRJNAME).runs/impl_1/procsys_wrapper_utilization_placed.rpt | grep "CLB LUTs" -B 3 -A 15
	cat $(BITFILE_PRJDIR)/$(BITFILE_PRJNAME).runs/impl_1/procsys_wrapper_utilization_placed.rpt | grep "RAMB36/FIFO" -B 4 -A 4
	cat $(BITFILE_PRJDIR)/$(BITFILE_PRJNAME).runs/impl_1/procsys_wrapper_utilization_placed.rpt | grep "DSP48E2 only" -B 4 -A 1
	cat $(BITFILE_PRJDIR)/$(BITFILE_PRJNAME).runs/impl_1/procsys_wrapper_timing_summary_postroute_physopted.rpt | grep "Design Timing Summary" -B 1 -A 10
