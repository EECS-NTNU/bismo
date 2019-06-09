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

if {$argc != 5} {
  puts "Expected: <bismo root> <accel verilog> <proj name> <proj dir> <freq>"
  exit
}

# pull cmdline variables to use during setup
set config_bismo_root  [lindex $argv 0]
set config_bismo_verilog "$config_bismo_root/fpga-tidbits/src/main/resources/verilog"
set config_bismo_vhdl "$config_bismo_root/src/main/vhdl"
set config_accel_verilog [lindex $argv 1]
set config_proj_name [lindex $argv 2]
set config_proj_dir [lindex $argv 3]
set config_freq [lindex $argv 4]
puts $config_bismo_verilog
# fixed for platform
set config_proj_part "xc7z020clg400-1"
set xdc_dir "$config_bismo_root/src/main/script/PYNQZ1/host"

# set up project
create_project $config_proj_name $config_proj_dir -part $config_proj_part
update_ip_catalog

# add the Verilog implementation for the accelerator
add_files -norecurse $config_accel_verilog
# add misc verilog files used by fpga-bismo
add_files -norecurse $config_bismo_verilog/Q_srl.v $config_bismo_verilog/DualPortBRAM.v
# add vhdl files used by fpga-bismo
add_files -norecurse $config_bismo_vhdl/

# create block design
create_bd_design "procsys"
create_bd_cell -type ip -vlnv xilinx.com:ip:processing_system7:5.5 ps7
set ps7 [get_bd_cells ps7]
apply_bd_automation -rule xilinx.com:bd_rule:processing_system7 -config {make_external "FIXED_IO, DDR" Master "Disable" Slave "Disable" } $ps7
source "${xdc_dir}/pynq_revC.tcl"

set_property -dict [apply_preset $ps7] $ps7
# enable AXI HP ports, set target frequency
set_property -dict [list CONFIG.PCW_USE_S_AXI_HP0 {1} CONFIG.PCW_USE_S_AXI_HP1 {0} CONFIG.PCW_USE_S_AXI_HP2 {0} CONFIG.PCW_USE_S_AXI_HP3 {0}] $ps7
# TODO expose top-level ports?
#set_property -dict [list CONFIG.PCW_FPGA0_PERIPHERAL_FREQMHZ $config_freq CONFIG.PCW_USE_S_AXI_HP0 {1} CONFIG.PCW_USE_S_AXI_HP1 {1} CONFIG.PCW_USE_S_AXI_HP2 {1} CONFIG.PCW_USE_S_AXI_HP3 {1}] [get_bd_cells processing_system7_0]
set_property -dict [list CONFIG.PCW_FPGA0_PERIPHERAL_FREQMHZ $config_freq CONFIG.PCW_FPGA1_PERIPHERAL_FREQMHZ {142.86} CONFIG.PCW_FPGA2_PERIPHERAL_FREQMHZ {200} CONFIG.PCW_FPGA3_PERIPHERAL_FREQMHZ {166.67} CONFIG.PCW_EN_CLK1_PORT {1} CONFIG.PCW_EN_CLK2_PORT {1} CONFIG.PCW_EN_CLK3_PORT {1} CONFIG.PCW_USE_M_AXI_GP0 {1}] $ps7

# add the accelerator RTL module into the block design
create_bd_cell -type module -reference PYNQZ1Wrapper PYNQZ1Wrapper_0
# connect control-status registers
apply_bd_automation -rule xilinx.com:bd_rule:axi4 -config {Master "/ps7/M_AXI_GP0" Clk "Auto" }  [get_bd_intf_pins PYNQZ1Wrapper_0/csr]

# connect AXI master ports
apply_bd_automation -rule xilinx.com:bd_rule:axi4 -config {Master "/PYNQZ1Wrapper_0/mem0" Clk "Auto" }  [get_bd_intf_pins ps7/S_AXI_HP0]
#apply_bd_automation -rule xilinx.com:bd_rule:axi4 -config {Master "/PYNQZ1Wrapper_0/mem1" Clk "Auto" }  [get_bd_intf_pins ps7/S_AXI_HP1]
#apply_bd_automation -rule xilinx.com:bd_rule:axi4 -config {Master "/PYNQZ1Wrapper_0/mem2" Clk "Auto" }  [get_bd_intf_pins ps7/S_AXI_HP2]
#apply_bd_automation -rule xilinx.com:bd_rule:axi4 -config {Master "/PYNQZ1Wrapper_0/mem3" Clk "Auto" }  [get_bd_intf_pins ps7/S_AXI_HP3]

# rewire reset port to use active-high
disconnect_bd_net [get_bd_nets rst_ps7*peripheral_aresetn] [get_bd_pins PYNQZ1Wrapper_0/reset]
connect_bd_net [get_bd_pins [get_bd_cells *rst_ps7*]/peripheral_reset] [get_bd_pins PYNQZ1Wrapper_0/reset]

# make the block design look prettier
regenerate_bd_layout
validate_bd_design
save_bd_design
# generate tcl for PYNQ, used to set fclk
write_bd_tcl $config_proj_dir/bismo.tcl
# use global mode (no out-of-context) for bd synthesis
#set_property synth_checkpoint_mode None [get_files $config_proj_dir/$config_proj_name.srcs/sources_1/bd/procsys/procsys.bd]

# create HDL wrapper
make_wrapper -files [get_files $config_proj_dir/$config_proj_name.srcs/sources_1/bd/procsys/procsys.bd] -top
add_files -norecurse $config_proj_dir/$config_proj_name.srcs/sources_1/bd/procsys/hdl/procsys_wrapper.v
update_compile_order -fileset sources_1
update_compile_order -fileset sim_1
set_property top procsys_wrapper [current_fileset]

# use manual compile order to ensure accel verilog is processed prior to block design
#update_compile_order -fileset sources_1
#set_property source_mgmt_mode DisplayOnly [current_project]


# set synthesis strategy
set_property strategy Flow_PerfOptimized_high [get_runs synth_1]

set_property STEPS.SYNTH_DESIGN.ARGS.DIRECTIVE AlternateRoutability [get_runs synth_1]
set_property STEPS.SYNTH_DESIGN.ARGS.RETIMING true [get_runs synth_1]

set_property strategy Performance_ExtraTimingOpt [get_runs impl_1]
set_property STEPS.OPT_DESIGN.ARGS.DIRECTIVE Explore [get_runs impl_1]
set_property STEPS.POST_ROUTE_PHYS_OPT_DESIGN.ARGS.DIRECTIVE AggressiveExplore [get_runs impl_1]
set_property STEPS.PHYS_OPT_DESIGN.ARGS.DIRECTIVE AggressiveExplore [get_runs impl_1]
set_property STEPS.POST_ROUTE_PHYS_OPT_DESIGN.IS_ENABLED true [get_runs impl_1]

# do not ignore failure-level VHDL assertions
set_param synth.elaboration.rodinMoreOptions {rt::set_parameter ignoreVhdlAssertStmts false}
