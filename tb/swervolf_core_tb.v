// SPDX-License-Identifier: Apache-2.0
// Copyright 2019 Western Digital Corporation or its affiliates.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
// http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

//********************************************************************************
// $Id$
//
// Function: Verilog testbench for SweRVolf
// Comments:
//
//********************************************************************************

`default_nettype none
module swervolf_core_tb
  #(parameter bootrom_file  = "",
  parameter MAX_CYCLES = 0)
`ifdef VERILATOR
  (input wire clk,
   input wire  rst,
   input wire  i_jtag_tck,
   input wire  i_jtag_tms,
   input wire  i_jtag_tdi,
   input wire  i_jtag_trst_n,
   output wire o_jtag_tdo,
   output wire o_uart_tx,
   output wire o_gpio)
`endif
  ;

`ifndef VERILATOR
   reg 	 clk = 1'b0;
   reg 	 rst = 1'b1;
   always #10 clk <= !clk;
   initial #100 rst <= 1'b0;
   wire  o_gpio;
   wire i_jtag_tck = 1'b0;
   wire i_jtag_tms = 1'b0;
   wire i_jtag_tdi = 1'b0;
   wire i_jtag_trst_n = 1'b0;
   wire o_jtag_tdo;
   wire  o_uart_tx;

   uart_decoder #(115200) uart_decoder (o_uart_tx);

`endif

   reg [1023:0] ram_init_file;

   initial begin
      if (|$test$plusargs("jtag_vpi_enable"))
          $display("JTAG VPI enabled. Not loading RAM");
      else if ($value$plusargs("ram_init_file=%s", ram_init_file)) begin
          $display("Loading RAM contents from %0s", ram_init_file);
          $readmemh(ram_init_file, vex.ram.ram.mem);
      end
   end

   reg [1023:0] rom_init_file;

   initial begin
      if ($value$plusargs("rom_init_file=%s", rom_init_file)) begin
          $display("Loading ROM contents from %0s", rom_init_file);
          $readmemh(rom_init_file, vex.bootrom.ram.mem);
      end else if (!(|bootrom_file))
	/*
	 Set mrac to 0xAAAA0000 and jump to address 0
	 if no bootloader is selected
	 0:   aaaa02b7                lui     t0,0xaaaa0
	 4:   7c029073                csrw    0x7c0,t0
	 8:   00000067                jr      zero

	 */
	vex.bootrom.ram.mem[0] = 64'h7c029073aaaa02b7;
	vex.bootrom.ram.mem[1] = 64'h0000000000000067;
   end

   wire [63:0] gpio_out;
   assign o_gpio = gpio_out[0];

  //  wire [5:0]  ram_awid;
  //  wire [31:0] ram_awaddr;
  //  wire [7:0]  ram_awlen;
  //  wire [2:0]  ram_awsize;
  //  wire [1:0]  ram_awburst;
  //  wire        ram_awlock;
  //  wire [3:0]  ram_awcache;
  //  wire [2:0]  ram_awprot;
  //  wire [3:0]  ram_awregion;
  //  wire [3:0]  ram_awqos;
  //  wire        ram_awvalid;
  //  wire        ram_awready;
  //  wire [5:0]  ram_arid;
  //  wire [31:0] ram_araddr;
  //  wire [7:0]  ram_arlen;
  //  wire [2:0]  ram_arsize;
  //  wire [1:0]  ram_arburst;
  //  wire        ram_arlock;
  //  wire [3:0]  ram_arcache;
  //  wire [2:0]  ram_arprot;
  //  wire [3:0]  ram_arregion;
  //  wire [3:0]  ram_arqos;
  //  wire        ram_arvalid;
  //  wire        ram_arready;
  //  wire [63:0] ram_wdata;
  //  wire [7:0]  ram_wstrb;
  //  wire        ram_wlast;
  //  wire        ram_wvalid;
  //  wire        ram_wready;
  //  wire [5:0]  ram_bid;
  //  wire [1:0]  ram_bresp;
  //  wire        ram_bvalid;
  //  wire        ram_bready;
  //  wire [5:0]  ram_rid;
  //  wire [63:0] ram_rdata;
  //  wire [1:0]  ram_rresp;
  //  wire        ram_rlast;
  //  wire        ram_rvalid;
  //  wire        ram_rready;

   wire        dmi_reg_en;
   wire [6:0]  dmi_reg_addr;
   wire        dmi_reg_wr_en;
   wire [31:0] dmi_reg_wdata;
   wire [31:0] dmi_reg_rdata;
   wire        dmi_hard_reset;

    wire           wb_m2s_ram_cyc;
    wire           wb_m2s_ram_stb;
    wire           wb_s2m_ram_ack;
    wire           wb_m2s_ram_we;
    // wire [31:2]    wb_m2s_ram_adr;
    wire [31:0]    wb_s2m_ram_dat;
    wire [31:0]    wb_m2s_ram_dat;
    wire [ 3:0]    wb_m2s_ram_sel;
    wire           wb_s2m_ram_ERR;
    wire [ 1:0]    wb_m2s_ram_BTE;
    wire [ 2:0]    wb_m2s_ram_CTI;

  

   vex_core
     #(.bootrom_file (bootrom_file),
       .clk_freq_hz (32'd50_000_000),
       .MAX_CYCLES(MAX_CYCLES))
   vex
     (.clk  (clk),
      .rstn (!rst),

      .i_gpio              (64'd0),
      .o_gpio              (gpio_out));

endmodule
