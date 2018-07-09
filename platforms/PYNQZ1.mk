BITFILE_PRJNAME := bitfile_synth
BITFILE_PRJDIR := $(BUILD_DIR)/bitfile_synth
GEN_BITFILE_PATH := $(BITFILE_PRJDIR)/$(BITFILE_PRJNAME).runs/impl_1/procsys_wrapper.bit
VIVADO_PROJ_SCRIPT := $(TOP)/src/main/script/$(PLATFORM)/host/make-vivado-project.tcl
VIVADO_SYNTH_SCRIPT := $(TOP)/src/main/script/$(PLATFORM)/host/synth-vivado-project.tcl

# create a new Vivado project
hw_vivadoproj: $(BITFILE_PRJDIR)/bitfile_synth.xpr

$(BITFILE_PRJDIR)/bitfile_synth.xpr: $(HW_VERILOG)
	vivado -mode $(VIVADO_MODE) -source $(VIVADO_PROJ_SCRIPT) -tclargs $(TOP) $(HW_VERILOG) $(BITFILE_PRJNAME) $(BITFILE_PRJDIR) $(FREQ_MHZ)

# launch Vivado in GUI mode with created project
launch_vivado_gui: $(BITFILE_PRJDIR)/bitfile_synth.xpr
	vivado -mode gui $(BITFILE_PRJDIR)/$(BITFILE_PRJNAME).xpr

# run bitfile synthesis for the Vivado project
bitfile: $(GEN_BITFILE_PATH)

$(GEN_BITFILE_PATH): $(BITFILE_PRJDIR)/bitfile_synth.xpr
	vivado -mode $(VIVADO_MODE) -source $(VIVADO_SYNTH_SCRIPT) -tclargs $(BITFILE_PRJDIR)/$(BITFILE_PRJNAME).xpr

# copy bitfile to the deployment folder, make an empty tcl script for bitfile loader
hw: $(GEN_BITFILE_PATH)
	mkdir -p $(BUILD_DIR_DEPLOY); cp $(GEN_BITFILE_PATH) $(BUILD_DIR_DEPLOY)/bismo.bit; touch $(BUILD_DIR_DEPLOY)/bismo.tcl
