diff --git a/src/main/script/PYNQU96/host/ultra96.xdc b/src/main/script/PYNQU96/host/ultra96.xdc
index 203ffba..4f57429 100644
--- a/src/main/script/PYNQU96/host/ultra96.xdc
+++ b/src/main/script/PYNQU96/host/ultra96.xdc
@@ -63,3 +63,106 @@ set_property IOSTANDARD LVCMOS18 [get_ports BT*]
 set_property PACKAGE_PIN B7 [get_ports BT_ctsn]
 #BT_HCI_CTS on FPGA / emio_uart0_rtsn
 set_property PACKAGE_PIN B5 [get_ports BT_rtsn]
+
+create_debug_core u_ila_0 ila
+set_property ALL_PROBE_SAME_MU true [get_debug_cores u_ila_0]
+set_property ALL_PROBE_SAME_MU_CNT 1 [get_debug_cores u_ila_0]
+set_property C_ADV_TRIGGER false [get_debug_cores u_ila_0]
+set_property C_DATA_DEPTH 1024 [get_debug_cores u_ila_0]
+set_property C_EN_STRG_QUAL false [get_debug_cores u_ila_0]
+set_property C_INPUT_PIPE_STAGES 0 [get_debug_cores u_ila_0]
+set_property C_TRIGIN_EN false [get_debug_cores u_ila_0]
+set_property C_TRIGOUT_EN false [get_debug_cores u_ila_0]
+set_property port_width 1 [get_debug_ports u_ila_0/clk]
+connect_debug_port u_ila_0/clk [get_nets [list procsys_i/zynq_ultra_ps_e_0/inst/pl_clk2]]
+set_property PROBE_TYPE DATA_AND_TRIGGER [get_debug_ports u_ila_0/probe0]
+set_property port_width 32 [get_debug_ports u_ila_0/probe0]
+connect_debug_port u_ila_0/probe0 [get_nets [list {procsys_i/PYNQU96Wrapper_0_mem0_ARADDR[0]} {procsys_i/PYNQU96Wrapper_0_mem0_ARADDR[1]} {procsys_i/PYNQU96Wrapper_0_mem0_ARADDR[2]} {procsys_i/PYNQU96Wrapper_0_mem0_ARADDR[3]} {procsys_i/PYNQU96Wrapper_0_mem0_ARADDR[4]} {procsys_i/PYNQU96Wrapper_0_mem0_ARADDR[5]} {procsys_i/PYNQU96Wrapper_0_mem0_ARADDR[6]} {procsys_i/PYNQU96Wrapper_0_mem0_ARADDR[7]} {procsys_i/PYNQU96Wrapper_0_mem0_ARADDR[8]} {procsys_i/PYNQU96Wrapper_0_mem0_ARADDR[9]} {procsys_i/PYNQU96Wrapper_0_mem0_ARADDR[10]} {procsys_i/PYNQU96Wrapper_0_mem0_ARADDR[11]} {procsys_i/PYNQU96Wrapper_0_mem0_ARADDR[12]} {procsys_i/PYNQU96Wrapper_0_mem0_ARADDR[13]} {procsys_i/PYNQU96Wrapper_0_mem0_ARADDR[14]} {procsys_i/PYNQU96Wrapper_0_mem0_ARADDR[15]} {procsys_i/PYNQU96Wrapper_0_mem0_ARADDR[16]} {procsys_i/PYNQU96Wrapper_0_mem0_ARADDR[17]} {procsys_i/PYNQU96Wrapper_0_mem0_ARADDR[18]} {procsys_i/PYNQU96Wrapper_0_mem0_ARADDR[19]} {procsys_i/PYNQU96Wrapper_0_mem0_ARADDR[20]} {procsys_i/PYNQU96Wrapper_0_mem0_ARADDR[21]} {procsys_i/PYNQU96Wrapper_0_mem0_ARADDR[22]} {procsys_i/PYNQU96Wrapper_0_mem0_ARADDR[23]} {procsys_i/PYNQU96Wrapper_0_mem0_ARADDR[24]} {procsys_i/PYNQU96Wrapper_0_mem0_ARADDR[25]} {procsys_i/PYNQU96Wrapper_0_mem0_ARADDR[26]} {procsys_i/PYNQU96Wrapper_0_mem0_ARADDR[27]} {procsys_i/PYNQU96Wrapper_0_mem0_ARADDR[28]} {procsys_i/PYNQU96Wrapper_0_mem0_ARADDR[29]} {procsys_i/PYNQU96Wrapper_0_mem0_ARADDR[30]} {procsys_i/PYNQU96Wrapper_0_mem0_ARADDR[31]}]]
+create_debug_port u_ila_0 probe
+set_property PROBE_TYPE DATA_AND_TRIGGER [get_debug_ports u_ila_0/probe1]
+set_property port_width 1 [get_debug_ports u_ila_0/probe1]
+connect_debug_port u_ila_0/probe1 [get_nets [list {procsys_i/PYNQU96Wrapper_0_mem0_ARID[0]}]]
+create_debug_port u_ila_0 probe
+set_property PROBE_TYPE DATA_AND_TRIGGER [get_debug_ports u_ila_0/probe2]
+set_property port_width 3 [get_debug_ports u_ila_0/probe2]
+connect_debug_port u_ila_0/probe2 [get_nets [list {procsys_i/PYNQU96Wrapper_0_mem0_ARLEN[0]} {procsys_i/PYNQU96Wrapper_0_mem0_ARLEN[3]} {procsys_i/PYNQU96Wrapper_0_mem0_ARLEN[4]}]]
+create_debug_port u_ila_0 probe
+set_property PROBE_TYPE DATA_AND_TRIGGER [get_debug_ports u_ila_0/probe3]
+set_property port_width 32 [get_debug_ports u_ila_0/probe3]
+connect_debug_port u_ila_0/probe3 [get_nets [list {procsys_i/PYNQU96Wrapper_0_mem0_AWADDR[0]} {procsys_i/PYNQU96Wrapper_0_mem0_AWADDR[1]} {procsys_i/PYNQU96Wrapper_0_mem0_AWADDR[2]} {procsys_i/PYNQU96Wrapper_0_mem0_AWADDR[3]} {procsys_i/PYNQU96Wrapper_0_mem0_AWADDR[4]} {procsys_i/PYNQU96Wrapper_0_mem0_AWADDR[5]} {procsys_i/PYNQU96Wrapper_0_mem0_AWADDR[6]} {procsys_i/PYNQU96Wrapper_0_mem0_AWADDR[7]} {procsys_i/PYNQU96Wrapper_0_mem0_AWADDR[8]} {procsys_i/PYNQU96Wrapper_0_mem0_AWADDR[9]} {procsys_i/PYNQU96Wrapper_0_mem0_AWADDR[10]} {procsys_i/PYNQU96Wrapper_0_mem0_AWADDR[11]} {procsys_i/PYNQU96Wrapper_0_mem0_AWADDR[12]} {procsys_i/PYNQU96Wrapper_0_mem0_AWADDR[13]} {procsys_i/PYNQU96Wrapper_0_mem0_AWADDR[14]} {procsys_i/PYNQU96Wrapper_0_mem0_AWADDR[15]} {procsys_i/PYNQU96Wrapper_0_mem0_AWADDR[16]} {procsys_i/PYNQU96Wrapper_0_mem0_AWADDR[17]} {procsys_i/PYNQU96Wrapper_0_mem0_AWADDR[18]} {procsys_i/PYNQU96Wrapper_0_mem0_AWADDR[19]} {procsys_i/PYNQU96Wrapper_0_mem0_AWADDR[20]} {procsys_i/PYNQU96Wrapper_0_mem0_AWADDR[21]} {procsys_i/PYNQU96Wrapper_0_mem0_AWADDR[22]} {procsys_i/PYNQU96Wrapper_0_mem0_AWADDR[23]} {procsys_i/PYNQU96Wrapper_0_mem0_AWADDR[24]} {procsys_i/PYNQU96Wrapper_0_mem0_AWADDR[25]} {procsys_i/PYNQU96Wrapper_0_mem0_AWADDR[26]} {procsys_i/PYNQU96Wrapper_0_mem0_AWADDR[27]} {procsys_i/PYNQU96Wrapper_0_mem0_AWADDR[28]} {procsys_i/PYNQU96Wrapper_0_mem0_AWADDR[29]} {procsys_i/PYNQU96Wrapper_0_mem0_AWADDR[30]} {procsys_i/PYNQU96Wrapper_0_mem0_AWADDR[31]}]]
+create_debug_port u_ila_0 probe
+set_property PROBE_TYPE DATA_AND_TRIGGER [get_debug_ports u_ila_0/probe4]
+set_property port_width 1 [get_debug_ports u_ila_0/probe4]
+connect_debug_port u_ila_0/probe4 [get_nets [list {procsys_i/PYNQU96Wrapper_0_mem0_AWID[0]}]]
+create_debug_port u_ila_0 probe
+set_property PROBE_TYPE DATA_AND_TRIGGER [get_debug_ports u_ila_0/probe5]
+set_property port_width 3 [get_debug_ports u_ila_0/probe5]
+connect_debug_port u_ila_0/probe5 [get_nets [list {procsys_i/PYNQU96Wrapper_0_mem0_AWLEN[0]} {procsys_i/PYNQU96Wrapper_0_mem0_AWLEN[3]} {procsys_i/PYNQU96Wrapper_0_mem0_AWLEN[4]}]]
+create_debug_port u_ila_0 probe
+set_property PROBE_TYPE DATA_AND_TRIGGER [get_debug_ports u_ila_0/probe6]
+set_property port_width 6 [get_debug_ports u_ila_0/probe6]
+connect_debug_port u_ila_0/probe6 [get_nets [list {procsys_i/PYNQU96Wrapper_0_mem0_BID[0]} {procsys_i/PYNQU96Wrapper_0_mem0_BID[1]} {procsys_i/PYNQU96Wrapper_0_mem0_BID[2]} {procsys_i/PYNQU96Wrapper_0_mem0_BID[3]} {procsys_i/PYNQU96Wrapper_0_mem0_BID[4]} {procsys_i/PYNQU96Wrapper_0_mem0_BID[5]}]]
+create_debug_port u_ila_0 probe
+set_property PROBE_TYPE DATA_AND_TRIGGER [get_debug_ports u_ila_0/probe7]
+set_property port_width 2 [get_debug_ports u_ila_0/probe7]
+connect_debug_port u_ila_0/probe7 [get_nets [list {procsys_i/PYNQU96Wrapper_0_mem0_BRESP[0]} {procsys_i/PYNQU96Wrapper_0_mem0_BRESP[1]}]]
+create_debug_port u_ila_0 probe
+set_property PROBE_TYPE DATA_AND_TRIGGER [get_debug_ports u_ila_0/probe8]
+set_property port_width 64 [get_debug_ports u_ila_0/probe8]
+connect_debug_port u_ila_0/probe8 [get_nets [list {procsys_i/PYNQU96Wrapper_0_mem0_RDATA[0]} {procsys_i/PYNQU96Wrapper_0_mem0_RDATA[1]} {procsys_i/PYNQU96Wrapper_0_mem0_RDATA[2]} {procsys_i/PYNQU96Wrapper_0_mem0_RDATA[3]} {procsys_i/PYNQU96Wrapper_0_mem0_RDATA[4]} {procsys_i/PYNQU96Wrapper_0_mem0_RDATA[5]} {procsys_i/PYNQU96Wrapper_0_mem0_RDATA[6]} {procsys_i/PYNQU96Wrapper_0_mem0_RDATA[7]} {procsys_i/PYNQU96Wrapper_0_mem0_RDATA[8]} {procsys_i/PYNQU96Wrapper_0_mem0_RDATA[9]} {procsys_i/PYNQU96Wrapper_0_mem0_RDATA[10]} {procsys_i/PYNQU96Wrapper_0_mem0_RDATA[11]} {procsys_i/PYNQU96Wrapper_0_mem0_RDATA[12]} {procsys_i/PYNQU96Wrapper_0_mem0_RDATA[13]} {procsys_i/PYNQU96Wrapper_0_mem0_RDATA[14]} {procsys_i/PYNQU96Wrapper_0_mem0_RDATA[15]} {procsys_i/PYNQU96Wrapper_0_mem0_RDATA[16]} {procsys_i/PYNQU96Wrapper_0_mem0_RDATA[17]} {procsys_i/PYNQU96Wrapper_0_mem0_RDATA[18]} {procsys_i/PYNQU96Wrapper_0_mem0_RDATA[19]} {procsys_i/PYNQU96Wrapper_0_mem0_RDATA[20]} {procsys_i/PYNQU96Wrapper_0_mem0_RDATA[21]} {procsys_i/PYNQU96Wrapper_0_mem0_RDATA[22]} {procsys_i/PYNQU96Wrapper_0_mem0_RDATA[23]} {procsys_i/PYNQU96Wrapper_0_mem0_RDATA[24]} {procsys_i/PYNQU96Wrapper_0_mem0_RDATA[25]} {procsys_i/PYNQU96Wrapper_0_mem0_RDATA[26]} {procsys_i/PYNQU96Wrapper_0_mem0_RDATA[27]} {procsys_i/PYNQU96Wrapper_0_mem0_RDATA[28]} {procsys_i/PYNQU96Wrapper_0_mem0_RDATA[29]} {procsys_i/PYNQU96Wrapper_0_mem0_RDATA[30]} {procsys_i/PYNQU96Wrapper_0_mem0_RDATA[31]} {procsys_i/PYNQU96Wrapper_0_mem0_RDATA[32]} {procsys_i/PYNQU96Wrapper_0_mem0_RDATA[33]} {procsys_i/PYNQU96Wrapper_0_mem0_RDATA[34]} {procsys_i/PYNQU96Wrapper_0_mem0_RDATA[35]} {procsys_i/PYNQU96Wrapper_0_mem0_RDATA[36]} {procsys_i/PYNQU96Wrapper_0_mem0_RDATA[37]} {procsys_i/PYNQU96Wrapper_0_mem0_RDATA[38]} {procsys_i/PYNQU96Wrapper_0_mem0_RDATA[39]} {procsys_i/PYNQU96Wrapper_0_mem0_RDATA[40]} {procsys_i/PYNQU96Wrapper_0_mem0_RDATA[41]} {procsys_i/PYNQU96Wrapper_0_mem0_RDATA[42]} {procsys_i/PYNQU96Wrapper_0_mem0_RDATA[43]} {procsys_i/PYNQU96Wrapper_0_mem0_RDATA[44]} {procsys_i/PYNQU96Wrapper_0_mem0_RDATA[45]} {procsys_i/PYNQU96Wrapper_0_mem0_RDATA[46]} {procsys_i/PYNQU96Wrapper_0_mem0_RDATA[47]} {procsys_i/PYNQU96Wrapper_0_mem0_RDATA[48]} {procsys_i/PYNQU96Wrapper_0_mem0_RDATA[49]} {procsys_i/PYNQU96Wrapper_0_mem0_RDATA[50]} {procsys_i/PYNQU96Wrapper_0_mem0_RDATA[51]} {procsys_i/PYNQU96Wrapper_0_mem0_RDATA[52]} {procsys_i/PYNQU96Wrapper_0_mem0_RDATA[53]} {procsys_i/PYNQU96Wrapper_0_mem0_RDATA[54]} {procsys_i/PYNQU96Wrapper_0_mem0_RDATA[55]} {procsys_i/PYNQU96Wrapper_0_mem0_RDATA[56]} {procsys_i/PYNQU96Wrapper_0_mem0_RDATA[57]} {procsys_i/PYNQU96Wrapper_0_mem0_RDATA[58]} {procsys_i/PYNQU96Wrapper_0_mem0_RDATA[59]} {procsys_i/PYNQU96Wrapper_0_mem0_RDATA[60]} {procsys_i/PYNQU96Wrapper_0_mem0_RDATA[61]} {procsys_i/PYNQU96Wrapper_0_mem0_RDATA[62]} {procsys_i/PYNQU96Wrapper_0_mem0_RDATA[63]}]]
+create_debug_port u_ila_0 probe
+set_property PROBE_TYPE DATA_AND_TRIGGER [get_debug_ports u_ila_0/probe9]
+set_property port_width 6 [get_debug_ports u_ila_0/probe9]
+connect_debug_port u_ila_0/probe9 [get_nets [list {procsys_i/PYNQU96Wrapper_0_mem0_RID[0]} {procsys_i/PYNQU96Wrapper_0_mem0_RID[1]} {procsys_i/PYNQU96Wrapper_0_mem0_RID[2]} {procsys_i/PYNQU96Wrapper_0_mem0_RID[3]} {procsys_i/PYNQU96Wrapper_0_mem0_RID[4]} {procsys_i/PYNQU96Wrapper_0_mem0_RID[5]}]]
+create_debug_port u_ila_0 probe
+set_property PROBE_TYPE DATA_AND_TRIGGER [get_debug_ports u_ila_0/probe10]
+set_property port_width 2 [get_debug_ports u_ila_0/probe10]
+connect_debug_port u_ila_0/probe10 [get_nets [list {procsys_i/PYNQU96Wrapper_0_mem0_RRESP[0]} {procsys_i/PYNQU96Wrapper_0_mem0_RRESP[1]}]]
+create_debug_port u_ila_0 probe
+set_property PROBE_TYPE DATA_AND_TRIGGER [get_debug_ports u_ila_0/probe11]
+set_property port_width 64 [get_debug_ports u_ila_0/probe11]
+connect_debug_port u_ila_0/probe11 [get_nets [list {procsys_i/PYNQU96Wrapper_0_mem0_WDATA[0]} {procsys_i/PYNQU96Wrapper_0_mem0_WDATA[1]} {procsys_i/PYNQU96Wrapper_0_mem0_WDATA[2]} {procsys_i/PYNQU96Wrapper_0_mem0_WDATA[3]} {procsys_i/PYNQU96Wrapper_0_mem0_WDATA[4]} {procsys_i/PYNQU96Wrapper_0_mem0_WDATA[5]} {procsys_i/PYNQU96Wrapper_0_mem0_WDATA[6]} {procsys_i/PYNQU96Wrapper_0_mem0_WDATA[7]} {procsys_i/PYNQU96Wrapper_0_mem0_WDATA[8]} {procsys_i/PYNQU96Wrapper_0_mem0_WDATA[9]} {procsys_i/PYNQU96Wrapper_0_mem0_WDATA[10]} {procsys_i/PYNQU96Wrapper_0_mem0_WDATA[11]} {procsys_i/PYNQU96Wrapper_0_mem0_WDATA[12]} {procsys_i/PYNQU96Wrapper_0_mem0_WDATA[13]} {procsys_i/PYNQU96Wrapper_0_mem0_WDATA[14]} {procsys_i/PYNQU96Wrapper_0_mem0_WDATA[15]} {procsys_i/PYNQU96Wrapper_0_mem0_WDATA[16]} {procsys_i/PYNQU96Wrapper_0_mem0_WDATA[17]} {procsys_i/PYNQU96Wrapper_0_mem0_WDATA[18]} {procsys_i/PYNQU96Wrapper_0_mem0_WDATA[19]} {procsys_i/PYNQU96Wrapper_0_mem0_WDATA[20]} {procsys_i/PYNQU96Wrapper_0_mem0_WDATA[21]} {procsys_i/PYNQU96Wrapper_0_mem0_WDATA[22]} {procsys_i/PYNQU96Wrapper_0_mem0_WDATA[23]} {procsys_i/PYNQU96Wrapper_0_mem0_WDATA[24]} {procsys_i/PYNQU96Wrapper_0_mem0_WDATA[25]} {procsys_i/PYNQU96Wrapper_0_mem0_WDATA[26]} {procsys_i/PYNQU96Wrapper_0_mem0_WDATA[27]} {procsys_i/PYNQU96Wrapper_0_mem0_WDATA[28]} {procsys_i/PYNQU96Wrapper_0_mem0_WDATA[29]} {procsys_i/PYNQU96Wrapper_0_mem0_WDATA[30]} {procsys_i/PYNQU96Wrapper_0_mem0_WDATA[31]} {procsys_i/PYNQU96Wrapper_0_mem0_WDATA[32]} {procsys_i/PYNQU96Wrapper_0_mem0_WDATA[33]} {procsys_i/PYNQU96Wrapper_0_mem0_WDATA[34]} {procsys_i/PYNQU96Wrapper_0_mem0_WDATA[35]} {procsys_i/PYNQU96Wrapper_0_mem0_WDATA[36]} {procsys_i/PYNQU96Wrapper_0_mem0_WDATA[37]} {procsys_i/PYNQU96Wrapper_0_mem0_WDATA[38]} {procsys_i/PYNQU96Wrapper_0_mem0_WDATA[39]} {procsys_i/PYNQU96Wrapper_0_mem0_WDATA[40]} {procsys_i/PYNQU96Wrapper_0_mem0_WDATA[41]} {procsys_i/PYNQU96Wrapper_0_mem0_WDATA[42]} {procsys_i/PYNQU96Wrapper_0_mem0_WDATA[43]} {procsys_i/PYNQU96Wrapper_0_mem0_WDATA[44]} {procsys_i/PYNQU96Wrapper_0_mem0_WDATA[45]} {procsys_i/PYNQU96Wrapper_0_mem0_WDATA[46]} {procsys_i/PYNQU96Wrapper_0_mem0_WDATA[47]} {procsys_i/PYNQU96Wrapper_0_mem0_WDATA[48]} {procsys_i/PYNQU96Wrapper_0_mem0_WDATA[49]} {procsys_i/PYNQU96Wrapper_0_mem0_WDATA[50]} {procsys_i/PYNQU96Wrapper_0_mem0_WDATA[51]} {procsys_i/PYNQU96Wrapper_0_mem0_WDATA[52]} {procsys_i/PYNQU96Wrapper_0_mem0_WDATA[53]} {procsys_i/PYNQU96Wrapper_0_mem0_WDATA[54]} {procsys_i/PYNQU96Wrapper_0_mem0_WDATA[55]} {procsys_i/PYNQU96Wrapper_0_mem0_WDATA[56]} {procsys_i/PYNQU96Wrapper_0_mem0_WDATA[57]} {procsys_i/PYNQU96Wrapper_0_mem0_WDATA[58]} {procsys_i/PYNQU96Wrapper_0_mem0_WDATA[59]} {procsys_i/PYNQU96Wrapper_0_mem0_WDATA[60]} {procsys_i/PYNQU96Wrapper_0_mem0_WDATA[61]} {procsys_i/PYNQU96Wrapper_0_mem0_WDATA[62]} {procsys_i/PYNQU96Wrapper_0_mem0_WDATA[63]}]]
+create_debug_port u_ila_0 probe
+set_property PROBE_TYPE DATA_AND_TRIGGER [get_debug_ports u_ila_0/probe12]
+set_property port_width 1 [get_debug_ports u_ila_0/probe12]
+connect_debug_port u_ila_0/probe12 [get_nets [list procsys_i/PYNQU96Wrapper_0_mem0_ARREADY]]
+create_debug_port u_ila_0 probe
+set_property PROBE_TYPE DATA_AND_TRIGGER [get_debug_ports u_ila_0/probe13]
+set_property port_width 1 [get_debug_ports u_ila_0/probe13]
+connect_debug_port u_ila_0/probe13 [get_nets [list procsys_i/PYNQU96Wrapper_0_mem0_ARVALID]]
+create_debug_port u_ila_0 probe
+set_property PROBE_TYPE DATA_AND_TRIGGER [get_debug_ports u_ila_0/probe14]
+set_property port_width 1 [get_debug_ports u_ila_0/probe14]
+connect_debug_port u_ila_0/probe14 [get_nets [list procsys_i/PYNQU96Wrapper_0_mem0_AWREADY]]
+create_debug_port u_ila_0 probe
+set_property PROBE_TYPE DATA_AND_TRIGGER [get_debug_ports u_ila_0/probe15]
+set_property port_width 1 [get_debug_ports u_ila_0/probe15]
+connect_debug_port u_ila_0/probe15 [get_nets [list procsys_i/PYNQU96Wrapper_0_mem0_AWVALID]]
+create_debug_port u_ila_0 probe
+set_property PROBE_TYPE DATA_AND_TRIGGER [get_debug_ports u_ila_0/probe16]
+set_property port_width 1 [get_debug_ports u_ila_0/probe16]
+connect_debug_port u_ila_0/probe16 [get_nets [list procsys_i/PYNQU96Wrapper_0_mem0_BVALID]]
+create_debug_port u_ila_0 probe
+set_property PROBE_TYPE DATA_AND_TRIGGER [get_debug_ports u_ila_0/probe17]
+set_property port_width 1 [get_debug_ports u_ila_0/probe17]
+connect_debug_port u_ila_0/probe17 [get_nets [list procsys_i/PYNQU96Wrapper_0_mem0_RLAST]]
+create_debug_port u_ila_0 probe
+set_property PROBE_TYPE DATA_AND_TRIGGER [get_debug_ports u_ila_0/probe18]
+set_property port_width 1 [get_debug_ports u_ila_0/probe18]
+connect_debug_port u_ila_0/probe18 [get_nets [list procsys_i/PYNQU96Wrapper_0_mem0_RREADY]]
+create_debug_port u_ila_0 probe
+set_property PROBE_TYPE DATA_AND_TRIGGER [get_debug_ports u_ila_0/probe19]
+set_property port_width 1 [get_debug_ports u_ila_0/probe19]
+connect_debug_port u_ila_0/probe19 [get_nets [list procsys_i/PYNQU96Wrapper_0_mem0_RVALID]]
+create_debug_port u_ila_0 probe
+set_property PROBE_TYPE DATA_AND_TRIGGER [get_debug_ports u_ila_0/probe20]
+set_property port_width 1 [get_debug_ports u_ila_0/probe20]
+connect_debug_port u_ila_0/probe20 [get_nets [list procsys_i/PYNQU96Wrapper_0_mem0_WREADY]]
+create_debug_port u_ila_0 probe
+set_property PROBE_TYPE DATA_AND_TRIGGER [get_debug_ports u_ila_0/probe21]
+set_property port_width 1 [get_debug_ports u_ila_0/probe21]
+connect_debug_port u_ila_0/probe21 [get_nets [list procsys_i/PYNQU96Wrapper_0_mem0_WVALID]]
+set_property C_CLK_INPUT_FREQ_HZ 300000000 [get_debug_cores dbg_hub]
+set_property C_ENABLE_CLK_DIVIDER false [get_debug_cores dbg_hub]
+set_property C_USER_SCAN_CHAIN 1 [get_debug_cores dbg_hub]
+connect_debug_port dbg_hub/clk [get_nets u_ila_0_pl_clk2]
