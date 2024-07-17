//Copyright 1986-2021 Xilinx, Inc. All Rights Reserved.
//--------------------------------------------------------------------------------
//Tool Version: Vivado v.2021.1 (lin64) Build 3247384 Thu Jun 10 19:36:07 MDT 2021
//Date        : Mon Jul  1 17:21:09 2024
//Host        : simtool-5 running 64-bit Ubuntu 20.04.6 LTS
//Command     : generate_target top_level.bd
//Design      : top_level
//Purpose     : IP block netlist
//--------------------------------------------------------------------------------
`timescale 1 ps / 1 ps

module axi_uart_bridge_imp_1OII57Q
   (M_AXI_araddr,
    M_AXI_arready,
    M_AXI_arvalid,
    M_AXI_awaddr,
    M_AXI_awready,
    M_AXI_awvalid,
    M_AXI_bready,
    M_AXI_bresp,
    M_AXI_bvalid,
    M_AXI_rdata,
    M_AXI_rready,
    M_AXI_rresp,
    M_AXI_rvalid,
    M_AXI_wdata,
    M_AXI_wready,
    M_AXI_wstrb,
    M_AXI_wvalid,
    UART_rxd,
    UART_txd,
    aclk,
    aresetn);
  output M_AXI_araddr;
  input [0:0]M_AXI_arready;
  output [0:0]M_AXI_arvalid;
  output M_AXI_awaddr;
  input [0:0]M_AXI_awready;
  output [0:0]M_AXI_awvalid;
  output [0:0]M_AXI_bready;
  input [1:0]M_AXI_bresp;
  input [0:0]M_AXI_bvalid;
  input M_AXI_rdata;
  output [0:0]M_AXI_rready;
  input [1:0]M_AXI_rresp;
  input [0:0]M_AXI_rvalid;
  output M_AXI_wdata;
  input [0:0]M_AXI_wready;
  output M_AXI_wstrb;
  output [0:0]M_AXI_wvalid;
  input UART_rxd;
  output UART_txd;
  input aclk;
  input aresetn;

  wire [63:0]Conn1_ARADDR;
  wire [0:0]Conn1_ARREADY;
  wire Conn1_ARVALID;
  wire [63:0]Conn1_AWADDR;
  wire [0:0]Conn1_AWREADY;
  wire Conn1_AWVALID;
  wire Conn1_BREADY;
  wire [1:0]Conn1_BRESP;
  wire [0:0]Conn1_BVALID;
  wire Conn1_RDATA;
  wire Conn1_RREADY;
  wire [1:0]Conn1_RRESP;
  wire [0:0]Conn1_RVALID;
  wire [31:0]Conn1_WDATA;
  wire [0:0]Conn1_WREADY;
  wire [3:0]Conn1_WSTRB;
  wire Conn1_WVALID;
  wire Conn2_RxD;
  wire Conn2_TxD;
  wire aclk_1;
  wire aresetn_1;
  wire [31:0]axi_uart_bridge_M_UART_ARADDR;
  wire axi_uart_bridge_M_UART_ARREADY;
  wire axi_uart_bridge_M_UART_ARVALID;
  wire [31:0]axi_uart_bridge_M_UART_AWADDR;
  wire axi_uart_bridge_M_UART_AWREADY;
  wire axi_uart_bridge_M_UART_AWVALID;
  wire axi_uart_bridge_M_UART_BREADY;
  wire [1:0]axi_uart_bridge_M_UART_BRESP;
  wire axi_uart_bridge_M_UART_BVALID;
  wire [31:0]axi_uart_bridge_M_UART_RDATA;
  wire axi_uart_bridge_M_UART_RREADY;
  wire [1:0]axi_uart_bridge_M_UART_RRESP;
  wire axi_uart_bridge_M_UART_RVALID;
  wire [31:0]axi_uart_bridge_M_UART_WDATA;
  wire axi_uart_bridge_M_UART_WREADY;
  wire [3:0]axi_uart_bridge_M_UART_WSTRB;
  wire axi_uart_bridge_M_UART_WVALID;
  wire axi_uartlite_interrupt;

  assign Conn1_ARREADY = M_AXI_arready[0];
  assign Conn1_AWREADY = M_AXI_awready[0];
  assign Conn1_BRESP = M_AXI_bresp[1:0];
  assign Conn1_BVALID = M_AXI_bvalid[0];
  assign Conn1_RDATA = M_AXI_rdata;
  assign Conn1_RRESP = M_AXI_rresp[1:0];
  assign Conn1_RVALID = M_AXI_rvalid[0];
  assign Conn1_WREADY = M_AXI_wready[0];
  assign Conn2_RxD = UART_rxd;
  assign M_AXI_araddr = Conn1_ARADDR[0];
  assign M_AXI_arvalid[0] = Conn1_ARVALID;
  assign M_AXI_awaddr = Conn1_AWADDR[0];
  assign M_AXI_awvalid[0] = Conn1_AWVALID;
  assign M_AXI_bready[0] = Conn1_BREADY;
  assign M_AXI_rready[0] = Conn1_RREADY;
  assign M_AXI_wdata = Conn1_WDATA[0];
  assign M_AXI_wstrb = Conn1_WSTRB[0];
  assign M_AXI_wvalid[0] = Conn1_WVALID;
  assign UART_txd = Conn2_TxD;
  assign aclk_1 = aclk;
  assign aresetn_1 = aresetn;
  top_level_axi_uart_bridge_0_0 axi_uart_bridge
       (.M_AXI_ARADDR(Conn1_ARADDR),
        .M_AXI_ARREADY(Conn1_ARREADY),
        .M_AXI_ARVALID(Conn1_ARVALID),
        .M_AXI_AWADDR(Conn1_AWADDR),
        .M_AXI_AWREADY(Conn1_AWREADY),
        .M_AXI_AWVALID(Conn1_AWVALID),
        .M_AXI_BREADY(Conn1_BREADY),
        .M_AXI_BRESP(Conn1_BRESP),
        .M_AXI_BVALID(Conn1_BVALID),
        .M_AXI_RDATA({Conn1_RDATA,Conn1_RDATA,Conn1_RDATA,Conn1_RDATA,Conn1_RDATA,Conn1_RDATA,Conn1_RDATA,Conn1_RDATA,Conn1_RDATA,Conn1_RDATA,Conn1_RDATA,Conn1_RDATA,Conn1_RDATA,Conn1_RDATA,Conn1_RDATA,Conn1_RDATA,Conn1_RDATA,Conn1_RDATA,Conn1_RDATA,Conn1_RDATA,Conn1_RDATA,Conn1_RDATA,Conn1_RDATA,Conn1_RDATA,Conn1_RDATA,Conn1_RDATA,Conn1_RDATA,Conn1_RDATA,Conn1_RDATA,Conn1_RDATA,Conn1_RDATA,Conn1_RDATA}),
        .M_AXI_RREADY(Conn1_RREADY),
        .M_AXI_RRESP(Conn1_RRESP),
        .M_AXI_RVALID(Conn1_RVALID),
        .M_AXI_WDATA(Conn1_WDATA),
        .M_AXI_WREADY(Conn1_WREADY),
        .M_AXI_WSTRB(Conn1_WSTRB),
        .M_AXI_WVALID(Conn1_WVALID),
        .M_UART_ARADDR(axi_uart_bridge_M_UART_ARADDR),
        .M_UART_ARREADY(axi_uart_bridge_M_UART_ARREADY),
        .M_UART_ARVALID(axi_uart_bridge_M_UART_ARVALID),
        .M_UART_AWADDR(axi_uart_bridge_M_UART_AWADDR),
        .M_UART_AWREADY(axi_uart_bridge_M_UART_AWREADY),
        .M_UART_AWVALID(axi_uart_bridge_M_UART_AWVALID),
        .M_UART_BREADY(axi_uart_bridge_M_UART_BREADY),
        .M_UART_BRESP(axi_uart_bridge_M_UART_BRESP),
        .M_UART_BVALID(axi_uart_bridge_M_UART_BVALID),
        .M_UART_RDATA(axi_uart_bridge_M_UART_RDATA),
        .M_UART_RREADY(axi_uart_bridge_M_UART_RREADY),
        .M_UART_RRESP(axi_uart_bridge_M_UART_RRESP),
        .M_UART_RVALID(axi_uart_bridge_M_UART_RVALID),
        .M_UART_WDATA(axi_uart_bridge_M_UART_WDATA),
        .M_UART_WREADY(axi_uart_bridge_M_UART_WREADY),
        .M_UART_WSTRB(axi_uart_bridge_M_UART_WSTRB),
        .M_UART_WVALID(axi_uart_bridge_M_UART_WVALID),
        .UART_INT(axi_uartlite_interrupt),
        .aclk(aclk_1),
        .aresetn(aresetn_1));
  top_level_axi_uartlite_0_0 axi_uartlite
       (.interrupt(axi_uartlite_interrupt),
        .rx(Conn2_RxD),
        .s_axi_aclk(aclk_1),
        .s_axi_araddr(axi_uart_bridge_M_UART_ARADDR[3:0]),
        .s_axi_aresetn(aresetn_1),
        .s_axi_arready(axi_uart_bridge_M_UART_ARREADY),
        .s_axi_arvalid(axi_uart_bridge_M_UART_ARVALID),
        .s_axi_awaddr(axi_uart_bridge_M_UART_AWADDR[3:0]),
        .s_axi_awready(axi_uart_bridge_M_UART_AWREADY),
        .s_axi_awvalid(axi_uart_bridge_M_UART_AWVALID),
        .s_axi_bready(axi_uart_bridge_M_UART_BREADY),
        .s_axi_bresp(axi_uart_bridge_M_UART_BRESP),
        .s_axi_bvalid(axi_uart_bridge_M_UART_BVALID),
        .s_axi_rdata(axi_uart_bridge_M_UART_RDATA),
        .s_axi_rready(axi_uart_bridge_M_UART_RREADY),
        .s_axi_rresp(axi_uart_bridge_M_UART_RRESP),
        .s_axi_rvalid(axi_uart_bridge_M_UART_RVALID),
        .s_axi_wdata(axi_uart_bridge_M_UART_WDATA),
        .s_axi_wready(axi_uart_bridge_M_UART_WREADY),
        .s_axi_wstrb(axi_uart_bridge_M_UART_WSTRB),
        .s_axi_wvalid(axi_uart_bridge_M_UART_WVALID),
        .tx(Conn2_TxD));
endmodule

module data_consumer_imp_HDPY22
   (AXIS_RX0_tdata,
    AXIS_RX0_tready,
    AXIS_RX0_tvalid,
    AXIS_RX1_tdata,
    AXIS_RX1_tready,
    AXIS_RX1_tvalid,
    clk,
    resetn);
  input [511:0]AXIS_RX0_tdata;
  output AXIS_RX0_tready;
  input AXIS_RX0_tvalid;
  input [511:0]AXIS_RX1_tdata;
  output AXIS_RX1_tready;
  input AXIS_RX1_tvalid;
  input clk;
  input resetn;

  wire [511:0]datapath_AXIS_OUT0_TDATA;
  wire datapath_AXIS_OUT0_TREADY;
  wire datapath_AXIS_OUT0_TVALID;
  wire [511:0]datapath_AXIS_OUT1_TDATA;
  wire datapath_AXIS_OUT1_TREADY;
  wire datapath_AXIS_OUT1_TVALID;
  wire source_100mhz_peripheral_aresetn;
  wire system_clock_clk_100mhz;

  assign AXIS_RX0_tready = datapath_AXIS_OUT1_TREADY;
  assign AXIS_RX1_tready = datapath_AXIS_OUT0_TREADY;
  assign datapath_AXIS_OUT0_TDATA = AXIS_RX1_tdata[511:0];
  assign datapath_AXIS_OUT0_TVALID = AXIS_RX1_tvalid;
  assign datapath_AXIS_OUT1_TDATA = AXIS_RX0_tdata[511:0];
  assign datapath_AXIS_OUT1_TVALID = AXIS_RX0_tvalid;
  assign source_100mhz_peripheral_aresetn = resetn;
  assign system_clock_clk_100mhz = clk;
  top_level_data_consumer_0_0 data_consumer_0
       (.AXIS_RX_TDATA(datapath_AXIS_OUT0_TDATA),
        .AXIS_RX_TKEEP({1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1}),
        .AXIS_RX_TLAST(1'b0),
        .AXIS_RX_TREADY(datapath_AXIS_OUT0_TREADY),
        .AXIS_RX_TVALID(datapath_AXIS_OUT0_TVALID),
        .clk(system_clock_clk_100mhz),
        .resetn(source_100mhz_peripheral_aresetn));
  top_level_data_consumer_0_1 data_consumer_1
       (.AXIS_RX_TDATA(datapath_AXIS_OUT1_TDATA),
        .AXIS_RX_TKEEP({1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1,1'b1}),
        .AXIS_RX_TLAST(1'b0),
        .AXIS_RX_TREADY(datapath_AXIS_OUT1_TREADY),
        .AXIS_RX_TVALID(datapath_AXIS_OUT1_TVALID),
        .clk(system_clock_clk_100mhz),
        .resetn(source_100mhz_peripheral_aresetn));
endmodule

module datapath_imp_1B7C4UL
   (AXIS_FD_tdata,
    AXIS_FD_tready,
    AXIS_FD_tvalid,
    AXIS_MD_tdata,
    AXIS_MD_tready,
    AXIS_MD_tvalid,
    AXIS_OUT0_tdata,
    AXIS_OUT0_tready,
    AXIS_OUT0_tvalid,
    AXIS_OUT1_tdata,
    AXIS_OUT1_tready,
    AXIS_OUT1_tvalid,
    FRAME_SIZE,
    clk,
    resetn);
  input [511:0]AXIS_FD_tdata;
  output AXIS_FD_tready;
  input AXIS_FD_tvalid;
  input [511:0]AXIS_MD_tdata;
  output AXIS_MD_tready;
  input AXIS_MD_tvalid;
  output [511:0]AXIS_OUT0_tdata;
  input AXIS_OUT0_tready;
  output AXIS_OUT0_tvalid;
  output [511:0]AXIS_OUT1_tdata;
  input AXIS_OUT1_tready;
  output AXIS_OUT1_tvalid;
  input [31:0]FRAME_SIZE;
  input clk;
  input resetn;

  wire [511:0]AXIS_FD_1_TDATA;
  wire AXIS_FD_1_TREADY;
  wire AXIS_FD_1_TVALID;
  wire [511:0]AXIS_MD_1_TDATA;
  wire AXIS_MD_1_TREADY;
  wire AXIS_MD_1_TVALID;

  assign AXIS_FD_1_TDATA = AXIS_FD_tdata[511:0];
  assign AXIS_FD_1_TREADY = AXIS_OUT1_tready;
  assign AXIS_FD_1_TVALID = AXIS_FD_tvalid;
  assign AXIS_FD_tready = AXIS_FD_1_TREADY;
  assign AXIS_MD_1_TDATA = AXIS_MD_tdata[511:0];
  assign AXIS_MD_1_TREADY = AXIS_OUT0_tready;
  assign AXIS_MD_1_TVALID = AXIS_MD_tvalid;
  assign AXIS_MD_tready = AXIS_MD_1_TREADY;
  assign AXIS_OUT0_tdata[511:0] = AXIS_MD_1_TDATA;
  assign AXIS_OUT0_tvalid = AXIS_MD_1_TVALID;
  assign AXIS_OUT1_tdata[511:0] = AXIS_FD_1_TDATA;
  assign AXIS_OUT1_tvalid = AXIS_FD_1_TVALID;
endmodule

module source_50mhz_imp_1DU1HTR
   (CLK100MHZ,
    CPU_RESETN,
    clk_50mhz,
    interconnect_aresetn,
    peripheral_aresetn);
  input CLK100MHZ;
  input CPU_RESETN;
  output clk_50mhz;
  output [0:0]interconnect_aresetn;
  output [0:0]peripheral_aresetn;

  wire CLK100MHZ_1;
  wire CPU_RESETN_1;
  wire system_clock_clk_100mhz;
  wire [0:0]system_reset_interconnect_aresetn;
  wire [0:0]system_reset_peripheral_aresetn;

  assign CLK100MHZ_1 = CLK100MHZ;
  assign CPU_RESETN_1 = CPU_RESETN;
  assign clk_50mhz = system_clock_clk_100mhz;
  assign interconnect_aresetn[0] = system_reset_interconnect_aresetn;
  assign peripheral_aresetn[0] = system_reset_peripheral_aresetn;
  top_level_clk_wiz_0_0 system_clock
       (.clk_50mhz(system_clock_clk_100mhz),
        .clk_in1(CLK100MHZ_1));
  top_level_proc_sys_reset_0_0 system_reset
       (.aux_reset_in(1'b1),
        .dcm_locked(1'b1),
        .ext_reset_in(CPU_RESETN_1),
        .interconnect_aresetn(system_reset_interconnect_aresetn),
        .mb_debug_sys_rst(1'b0),
        .peripheral_aresetn(system_reset_peripheral_aresetn),
        .slowest_sync_clk(system_clock_clk_100mhz));
endmodule

(* CORE_GENERATION_INFO = "top_level,IP_Integrator,{x_ipVendor=xilinx.com,x_ipLibrary=BlockDiagram,x_ipName=top_level,x_ipVersion=1.00.a,x_ipLanguage=VERILOG,numBlks=13,numReposBlks=9,numNonXlnxBlks=0,numHierBlks=4,maxHierDepth=1,numSysgenBlks=0,numHlsBlks=0,numHdlrefBlks=4,numPkgbdBlks=0,bdsource=USER,da_axi4_cnt=1,synth_mode=OOC_per_IP}" *) (* HW_HANDOFF = "top_level.hwdef" *) 
module top_level
   (CLK100MHZ,
    CPU_RESETN,
    UART_rxd,
    UART_txd);
  (* X_INTERFACE_INFO = "xilinx.com:signal:clock:1.0 CLK.CLK100MHZ CLK" *) (* X_INTERFACE_PARAMETER = "XIL_INTERFACENAME CLK.CLK100MHZ, ASSOCIATED_RESET CPU_RESETN, CLK_DOMAIN top_level_CLK100MHZ, FREQ_HZ 100000000, FREQ_TOLERANCE_HZ 0, INSERT_VIP 0, PHASE 0.0" *) input CLK100MHZ;
  (* X_INTERFACE_INFO = "xilinx.com:signal:reset:1.0 RST.CPU_RESETN RST" *) (* X_INTERFACE_PARAMETER = "XIL_INTERFACENAME RST.CPU_RESETN, INSERT_VIP 0, POLARITY ACTIVE_LOW" *) input CPU_RESETN;
  (* X_INTERFACE_INFO = "xilinx.com:interface:uart:1.0 UART RxD" *) input UART_rxd;
  (* X_INTERFACE_INFO = "xilinx.com:interface:uart:1.0 UART TxD" *) output UART_txd;

  wire CLK100MHZ_1;
  wire CPU_RESETN_1;
  (* CONN_BUS_INFO = "datapath_AXIS_OUT0 xilinx.com:interface:axis:1.0 None TDATA" *) (* DONT_TOUCH *) wire [511:0]datapath_AXIS_OUT0_TDATA;
  (* CONN_BUS_INFO = "datapath_AXIS_OUT0 xilinx.com:interface:axis:1.0 None TREADY" *) (* DONT_TOUCH *) wire datapath_AXIS_OUT0_TREADY;
  (* CONN_BUS_INFO = "datapath_AXIS_OUT0 xilinx.com:interface:axis:1.0 None TVALID" *) (* DONT_TOUCH *) wire datapath_AXIS_OUT0_TVALID;
  (* CONN_BUS_INFO = "datapath_AXIS_OUT1 xilinx.com:interface:axis:1.0 None TDATA" *) (* DONT_TOUCH *) wire [511:0]datapath_AXIS_OUT1_TDATA;
  (* CONN_BUS_INFO = "datapath_AXIS_OUT1 xilinx.com:interface:axis:1.0 None TREADY" *) (* DONT_TOUCH *) wire datapath_AXIS_OUT1_TREADY;
  (* CONN_BUS_INFO = "datapath_AXIS_OUT1 xilinx.com:interface:axis:1.0 None TVALID" *) (* DONT_TOUCH *) wire datapath_AXIS_OUT1_TVALID;
  wire [511:0]frame_generator_AXIS_FD_TDATA;
  wire frame_generator_AXIS_FD_TREADY;
  wire frame_generator_AXIS_FD_TVALID;
  wire [511:0]frame_generator_AXIS_MD_TDATA;
  wire frame_generator_AXIS_MD_TREADY;
  wire frame_generator_AXIS_MD_TVALID;
  wire [31:0]frame_generator_FRAME_SIZE;
  wire hier_0_M_AXI_ARADDR;
  wire [0:0]hier_0_M_AXI_ARREADY;
  wire [0:0]hier_0_M_AXI_ARVALID;
  wire hier_0_M_AXI_AWADDR;
  wire [0:0]hier_0_M_AXI_AWREADY;
  wire [0:0]hier_0_M_AXI_AWVALID;
  wire [0:0]hier_0_M_AXI_BREADY;
  wire [1:0]hier_0_M_AXI_BRESP;
  wire [0:0]hier_0_M_AXI_BVALID;
  wire hier_0_M_AXI_RDATA;
  wire [0:0]hier_0_M_AXI_RREADY;
  wire [1:0]hier_0_M_AXI_RRESP;
  wire [0:0]hier_0_M_AXI_RVALID;
  wire hier_0_M_AXI_WDATA;
  wire [0:0]hier_0_M_AXI_WREADY;
  wire hier_0_M_AXI_WSTRB;
  wire [0:0]hier_0_M_AXI_WVALID;
  wire hier_0_UART_RxD;
  wire hier_0_UART_TxD;
  wire [0:0]source_100mhz_interconnect_aresetn;
  wire [0:0]source_100mhz_peripheral_aresetn;
  wire system_clock_clk_100mhz;
  wire system_interconnect_M00_AXI_ARADDR;
  wire [2:0]system_interconnect_M00_AXI_ARPROT;
  wire system_interconnect_M00_AXI_ARREADY;
  wire [0:0]system_interconnect_M00_AXI_ARVALID;
  wire system_interconnect_M00_AXI_AWADDR;
  wire [2:0]system_interconnect_M00_AXI_AWPROT;
  wire system_interconnect_M00_AXI_AWREADY;
  wire [0:0]system_interconnect_M00_AXI_AWVALID;
  wire [0:0]system_interconnect_M00_AXI_BREADY;
  wire [1:0]system_interconnect_M00_AXI_BRESP;
  wire system_interconnect_M00_AXI_BVALID;
  wire [31:0]system_interconnect_M00_AXI_RDATA;
  wire [0:0]system_interconnect_M00_AXI_RREADY;
  wire [1:0]system_interconnect_M00_AXI_RRESP;
  wire system_interconnect_M00_AXI_RVALID;
  wire system_interconnect_M00_AXI_WDATA;
  wire system_interconnect_M00_AXI_WREADY;
  wire system_interconnect_M00_AXI_WSTRB;
  wire [0:0]system_interconnect_M00_AXI_WVALID;

  assign CLK100MHZ_1 = CLK100MHZ;
  assign CPU_RESETN_1 = CPU_RESETN;
  assign UART_txd = hier_0_UART_TxD;
  assign hier_0_UART_RxD = UART_rxd;
  axi_uart_bridge_imp_1OII57Q axi_uart_bridge
       (.M_AXI_araddr(hier_0_M_AXI_ARADDR),
        .M_AXI_arready(hier_0_M_AXI_ARREADY),
        .M_AXI_arvalid(hier_0_M_AXI_ARVALID),
        .M_AXI_awaddr(hier_0_M_AXI_AWADDR),
        .M_AXI_awready(hier_0_M_AXI_AWREADY),
        .M_AXI_awvalid(hier_0_M_AXI_AWVALID),
        .M_AXI_bready(hier_0_M_AXI_BREADY),
        .M_AXI_bresp(hier_0_M_AXI_BRESP),
        .M_AXI_bvalid(hier_0_M_AXI_BVALID),
        .M_AXI_rdata(hier_0_M_AXI_RDATA),
        .M_AXI_rready(hier_0_M_AXI_RREADY),
        .M_AXI_rresp(hier_0_M_AXI_RRESP),
        .M_AXI_rvalid(hier_0_M_AXI_RVALID),
        .M_AXI_wdata(hier_0_M_AXI_WDATA),
        .M_AXI_wready(hier_0_M_AXI_WREADY),
        .M_AXI_wstrb(hier_0_M_AXI_WSTRB),
        .M_AXI_wvalid(hier_0_M_AXI_WVALID),
        .UART_rxd(hier_0_UART_RxD),
        .UART_txd(hier_0_UART_TxD),
        .aclk(system_clock_clk_100mhz),
        .aresetn(source_100mhz_peripheral_aresetn));
  data_consumer_imp_HDPY22 data_consumer
       (.AXIS_RX0_tdata(datapath_AXIS_OUT1_TDATA),
        .AXIS_RX0_tready(datapath_AXIS_OUT1_TREADY),
        .AXIS_RX0_tvalid(datapath_AXIS_OUT1_TVALID),
        .AXIS_RX1_tdata(datapath_AXIS_OUT0_TDATA),
        .AXIS_RX1_tready(datapath_AXIS_OUT0_TREADY),
        .AXIS_RX1_tvalid(datapath_AXIS_OUT0_TVALID),
        .clk(system_clock_clk_100mhz),
        .resetn(source_100mhz_peripheral_aresetn));
  datapath_imp_1B7C4UL datapath
       (.AXIS_FD_tdata(frame_generator_AXIS_FD_TDATA),
        .AXIS_FD_tready(frame_generator_AXIS_FD_TREADY),
        .AXIS_FD_tvalid(frame_generator_AXIS_FD_TVALID),
        .AXIS_MD_tdata(frame_generator_AXIS_MD_TDATA),
        .AXIS_MD_tready(frame_generator_AXIS_MD_TREADY),
        .AXIS_MD_tvalid(frame_generator_AXIS_MD_TVALID),
        .AXIS_OUT0_tdata(datapath_AXIS_OUT0_TDATA),
        .AXIS_OUT0_tready(datapath_AXIS_OUT0_TREADY),
        .AXIS_OUT0_tvalid(datapath_AXIS_OUT0_TVALID),
        .AXIS_OUT1_tdata(datapath_AXIS_OUT1_TDATA),
        .AXIS_OUT1_tready(datapath_AXIS_OUT1_TREADY),
        .AXIS_OUT1_tvalid(datapath_AXIS_OUT1_TVALID),
        .FRAME_SIZE(frame_generator_FRAME_SIZE),
        .clk(system_clock_clk_100mhz),
        .resetn(source_100mhz_peripheral_aresetn));
  top_level_frame_gen_0_0 frame_generator
       (.AXIS_FD_TDATA(frame_generator_AXIS_FD_TDATA),
        .AXIS_FD_TREADY(frame_generator_AXIS_FD_TREADY),
        .AXIS_FD_TVALID(frame_generator_AXIS_FD_TVALID),
        .AXIS_MD_TDATA(frame_generator_AXIS_MD_TDATA),
        .AXIS_MD_TREADY(frame_generator_AXIS_MD_TREADY),
        .AXIS_MD_TVALID(frame_generator_AXIS_MD_TVALID),
        .FRAME_SIZE(frame_generator_FRAME_SIZE),
        .S_AXI_ARADDR({system_interconnect_M00_AXI_ARADDR,system_interconnect_M00_AXI_ARADDR,system_interconnect_M00_AXI_ARADDR,system_interconnect_M00_AXI_ARADDR,system_interconnect_M00_AXI_ARADDR,system_interconnect_M00_AXI_ARADDR,system_interconnect_M00_AXI_ARADDR,system_interconnect_M00_AXI_ARADDR,system_interconnect_M00_AXI_ARADDR,system_interconnect_M00_AXI_ARADDR,system_interconnect_M00_AXI_ARADDR,system_interconnect_M00_AXI_ARADDR,system_interconnect_M00_AXI_ARADDR,system_interconnect_M00_AXI_ARADDR,system_interconnect_M00_AXI_ARADDR,system_interconnect_M00_AXI_ARADDR,system_interconnect_M00_AXI_ARADDR,system_interconnect_M00_AXI_ARADDR,system_interconnect_M00_AXI_ARADDR,system_interconnect_M00_AXI_ARADDR,system_interconnect_M00_AXI_ARADDR,system_interconnect_M00_AXI_ARADDR,system_interconnect_M00_AXI_ARADDR,system_interconnect_M00_AXI_ARADDR,system_interconnect_M00_AXI_ARADDR,system_interconnect_M00_AXI_ARADDR,system_interconnect_M00_AXI_ARADDR,system_interconnect_M00_AXI_ARADDR,system_interconnect_M00_AXI_ARADDR,system_interconnect_M00_AXI_ARADDR,system_interconnect_M00_AXI_ARADDR,system_interconnect_M00_AXI_ARADDR}),
        .S_AXI_ARPROT(system_interconnect_M00_AXI_ARPROT),
        .S_AXI_ARREADY(system_interconnect_M00_AXI_ARREADY),
        .S_AXI_ARVALID(system_interconnect_M00_AXI_ARVALID),
        .S_AXI_AWADDR({system_interconnect_M00_AXI_AWADDR,system_interconnect_M00_AXI_AWADDR,system_interconnect_M00_AXI_AWADDR,system_interconnect_M00_AXI_AWADDR,system_interconnect_M00_AXI_AWADDR,system_interconnect_M00_AXI_AWADDR,system_interconnect_M00_AXI_AWADDR,system_interconnect_M00_AXI_AWADDR,system_interconnect_M00_AXI_AWADDR,system_interconnect_M00_AXI_AWADDR,system_interconnect_M00_AXI_AWADDR,system_interconnect_M00_AXI_AWADDR,system_interconnect_M00_AXI_AWADDR,system_interconnect_M00_AXI_AWADDR,system_interconnect_M00_AXI_AWADDR,system_interconnect_M00_AXI_AWADDR,system_interconnect_M00_AXI_AWADDR,system_interconnect_M00_AXI_AWADDR,system_interconnect_M00_AXI_AWADDR,system_interconnect_M00_AXI_AWADDR,system_interconnect_M00_AXI_AWADDR,system_interconnect_M00_AXI_AWADDR,system_interconnect_M00_AXI_AWADDR,system_interconnect_M00_AXI_AWADDR,system_interconnect_M00_AXI_AWADDR,system_interconnect_M00_AXI_AWADDR,system_interconnect_M00_AXI_AWADDR,system_interconnect_M00_AXI_AWADDR,system_interconnect_M00_AXI_AWADDR,system_interconnect_M00_AXI_AWADDR,system_interconnect_M00_AXI_AWADDR,system_interconnect_M00_AXI_AWADDR}),
        .S_AXI_AWPROT(system_interconnect_M00_AXI_AWPROT),
        .S_AXI_AWREADY(system_interconnect_M00_AXI_AWREADY),
        .S_AXI_AWVALID(system_interconnect_M00_AXI_AWVALID),
        .S_AXI_BREADY(system_interconnect_M00_AXI_BREADY),
        .S_AXI_BRESP(system_interconnect_M00_AXI_BRESP),
        .S_AXI_BVALID(system_interconnect_M00_AXI_BVALID),
        .S_AXI_RDATA(system_interconnect_M00_AXI_RDATA),
        .S_AXI_RREADY(system_interconnect_M00_AXI_RREADY),
        .S_AXI_RRESP(system_interconnect_M00_AXI_RRESP),
        .S_AXI_RVALID(system_interconnect_M00_AXI_RVALID),
        .S_AXI_WDATA({system_interconnect_M00_AXI_WDATA,system_interconnect_M00_AXI_WDATA,system_interconnect_M00_AXI_WDATA,system_interconnect_M00_AXI_WDATA,system_interconnect_M00_AXI_WDATA,system_interconnect_M00_AXI_WDATA,system_interconnect_M00_AXI_WDATA,system_interconnect_M00_AXI_WDATA,system_interconnect_M00_AXI_WDATA,system_interconnect_M00_AXI_WDATA,system_interconnect_M00_AXI_WDATA,system_interconnect_M00_AXI_WDATA,system_interconnect_M00_AXI_WDATA,system_interconnect_M00_AXI_WDATA,system_interconnect_M00_AXI_WDATA,system_interconnect_M00_AXI_WDATA,system_interconnect_M00_AXI_WDATA,system_interconnect_M00_AXI_WDATA,system_interconnect_M00_AXI_WDATA,system_interconnect_M00_AXI_WDATA,system_interconnect_M00_AXI_WDATA,system_interconnect_M00_AXI_WDATA,system_interconnect_M00_AXI_WDATA,system_interconnect_M00_AXI_WDATA,system_interconnect_M00_AXI_WDATA,system_interconnect_M00_AXI_WDATA,system_interconnect_M00_AXI_WDATA,system_interconnect_M00_AXI_WDATA,system_interconnect_M00_AXI_WDATA,system_interconnect_M00_AXI_WDATA,system_interconnect_M00_AXI_WDATA,system_interconnect_M00_AXI_WDATA}),
        .S_AXI_WREADY(system_interconnect_M00_AXI_WREADY),
        .S_AXI_WSTRB({system_interconnect_M00_AXI_WSTRB,system_interconnect_M00_AXI_WSTRB,system_interconnect_M00_AXI_WSTRB,system_interconnect_M00_AXI_WSTRB}),
        .S_AXI_WVALID(system_interconnect_M00_AXI_WVALID),
        .clk(system_clock_clk_100mhz),
        .resetn(source_100mhz_peripheral_aresetn));
  source_50mhz_imp_1DU1HTR source_50mhz
       (.CLK100MHZ(CLK100MHZ_1),
        .CPU_RESETN(CPU_RESETN_1),
        .clk_50mhz(system_clock_clk_100mhz),
        .interconnect_aresetn(source_100mhz_interconnect_aresetn),
        .peripheral_aresetn(source_100mhz_peripheral_aresetn));
  top_level_system_ila_0_0 system_ila_0
       (.SLOT_0_AXIS_tdata(datapath_AXIS_OUT1_TDATA[0]),
        .SLOT_0_AXIS_tdest(1'b0),
        .SLOT_0_AXIS_tid(1'b0),
        .SLOT_0_AXIS_tkeep(1'b1),
        .SLOT_0_AXIS_tlast(1'b0),
        .SLOT_0_AXIS_tready(datapath_AXIS_OUT1_TREADY),
        .SLOT_0_AXIS_tstrb(1'b1),
        .SLOT_0_AXIS_tuser(1'b0),
        .SLOT_0_AXIS_tvalid(datapath_AXIS_OUT1_TVALID),
        .SLOT_1_AXIS_tdata(datapath_AXIS_OUT0_TDATA[0]),
        .SLOT_1_AXIS_tdest(1'b0),
        .SLOT_1_AXIS_tid(1'b0),
        .SLOT_1_AXIS_tkeep(1'b1),
        .SLOT_1_AXIS_tlast(1'b0),
        .SLOT_1_AXIS_tready(datapath_AXIS_OUT0_TREADY),
        .SLOT_1_AXIS_tstrb(1'b1),
        .SLOT_1_AXIS_tuser(1'b0),
        .SLOT_1_AXIS_tvalid(datapath_AXIS_OUT0_TVALID),
        .clk(system_clock_clk_100mhz),
        .resetn(1'b0));
  top_level_smartconnect_0_0 system_interconnect
       (.M00_AXI_araddr(system_interconnect_M00_AXI_ARADDR),
        .M00_AXI_arprot(system_interconnect_M00_AXI_ARPROT),
        .M00_AXI_arready(system_interconnect_M00_AXI_ARREADY),
        .M00_AXI_arvalid(system_interconnect_M00_AXI_ARVALID),
        .M00_AXI_awaddr(system_interconnect_M00_AXI_AWADDR),
        .M00_AXI_awprot(system_interconnect_M00_AXI_AWPROT),
        .M00_AXI_awready(system_interconnect_M00_AXI_AWREADY),
        .M00_AXI_awvalid(system_interconnect_M00_AXI_AWVALID),
        .M00_AXI_bid(1'b0),
        .M00_AXI_bready(system_interconnect_M00_AXI_BREADY),
        .M00_AXI_bresp(system_interconnect_M00_AXI_BRESP),
        .M00_AXI_buser(1'b0),
        .M00_AXI_bvalid(system_interconnect_M00_AXI_BVALID),
        .M00_AXI_rdata(system_interconnect_M00_AXI_RDATA[0]),
        .M00_AXI_rid(1'b0),
        .M00_AXI_rlast(1'b0),
        .M00_AXI_rready(system_interconnect_M00_AXI_RREADY),
        .M00_AXI_rresp(system_interconnect_M00_AXI_RRESP),
        .M00_AXI_ruser(1'b0),
        .M00_AXI_rvalid(system_interconnect_M00_AXI_RVALID),
        .M00_AXI_wdata(system_interconnect_M00_AXI_WDATA),
        .M00_AXI_wready(system_interconnect_M00_AXI_WREADY),
        .M00_AXI_wstrb(system_interconnect_M00_AXI_WSTRB),
        .M00_AXI_wvalid(system_interconnect_M00_AXI_WVALID),
        .M01_AXI_arready(1'b0),
        .M01_AXI_awready(1'b0),
        .M01_AXI_bid(1'b0),
        .M01_AXI_bresp({1'b0,1'b0}),
        .M01_AXI_buser(1'b0),
        .M01_AXI_bvalid(1'b0),
        .M01_AXI_rdata(1'b0),
        .M01_AXI_rid(1'b0),
        .M01_AXI_rlast(1'b0),
        .M01_AXI_rresp({1'b0,1'b0}),
        .M01_AXI_ruser(1'b0),
        .M01_AXI_rvalid(1'b0),
        .M01_AXI_wready(1'b0),
        .S00_AXI_araddr(hier_0_M_AXI_ARADDR),
        .S00_AXI_arburst({1'b0,1'b1}),
        .S00_AXI_arcache({1'b0,1'b0,1'b1,1'b1}),
        .S00_AXI_arid(1'b0),
        .S00_AXI_arlen(1'b0),
        .S00_AXI_arlock(1'b0),
        .S00_AXI_arprot({1'b0,1'b0,1'b0}),
        .S00_AXI_arqos({1'b0,1'b0,1'b0,1'b0}),
        .S00_AXI_arready(hier_0_M_AXI_ARREADY),
        .S00_AXI_arregion({1'b0,1'b0,1'b0,1'b0}),
        .S00_AXI_arsize({1'b0,1'b1,1'b0}),
        .S00_AXI_aruser(1'b0),
        .S00_AXI_arvalid(hier_0_M_AXI_ARVALID),
        .S00_AXI_awaddr(hier_0_M_AXI_AWADDR),
        .S00_AXI_awburst({1'b0,1'b1}),
        .S00_AXI_awcache({1'b0,1'b0,1'b1,1'b1}),
        .S00_AXI_awid(1'b0),
        .S00_AXI_awlen(1'b0),
        .S00_AXI_awlock(1'b0),
        .S00_AXI_awprot({1'b0,1'b0,1'b0}),
        .S00_AXI_awqos({1'b0,1'b0,1'b0,1'b0}),
        .S00_AXI_awready(hier_0_M_AXI_AWREADY),
        .S00_AXI_awregion({1'b0,1'b0,1'b0,1'b0}),
        .S00_AXI_awsize({1'b0,1'b1,1'b0}),
        .S00_AXI_awuser(1'b0),
        .S00_AXI_awvalid(hier_0_M_AXI_AWVALID),
        .S00_AXI_bready(hier_0_M_AXI_BREADY),
        .S00_AXI_bresp(hier_0_M_AXI_BRESP),
        .S00_AXI_bvalid(hier_0_M_AXI_BVALID),
        .S00_AXI_rdata(hier_0_M_AXI_RDATA),
        .S00_AXI_rready(hier_0_M_AXI_RREADY),
        .S00_AXI_rresp(hier_0_M_AXI_RRESP),
        .S00_AXI_rvalid(hier_0_M_AXI_RVALID),
        .S00_AXI_wdata(hier_0_M_AXI_WDATA),
        .S00_AXI_wid(1'b0),
        .S00_AXI_wlast(1'b0),
        .S00_AXI_wready(hier_0_M_AXI_WREADY),
        .S00_AXI_wstrb(hier_0_M_AXI_WSTRB),
        .S00_AXI_wuser(1'b0),
        .S00_AXI_wvalid(hier_0_M_AXI_WVALID),
        .aclk(system_clock_clk_100mhz),
        .aresetn(source_100mhz_interconnect_aresetn));
endmodule
