// SPDX-License-Identifier: Apache-2.0
// Copyright 2019-2020 Western Digital Corporation or its affiliates.
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
// Function: SweRVolf tech-agnostic toplevel
// Comments:
//
//********************************************************************************

`default_nettype none
module vex_core
  #(parameter bootrom_file  = "",
    parameter clk_freq_hz = 0,
    parameter MAX_CYCLES = 0,
    parameter RAM_SIZE = 32'h7FFFFFFF)
   (input wire 	clk,
    input wire 	       rstn,
   
    input wire [63:0]  i_gpio,
    output wire [63:0] o_gpio,
  
   input wire  i_jtag_tck,
   input wire  i_jtag_tms,
   input wire  i_jtag_tdi,
   input wire  i_jtag_trst_n,
   output wire o_jtag_tdo,

  output wire debug_resetOut);


   localparam BOOTROM_SIZE = 32'h1000;

   wire        rst_n = rstn;
   wire        timer_irq;
   wire        uart_irq;
   wire        eth_irq;
   wire        spi0_irq;
   wire        sw_irq4;
   wire        sw_irq3;
   wire        nmi_int;

   wire [31:0] nmi_vec;

   wire 		      wb_clk = clk;
   wire 		      wb_rst = ~rst_n;

`include "wb_intercon.vh"

   wire [15:2] 		       wb_adr;


  //  assign wb_m2s_wbi_cti = 3'b000;
  //  assign wb_m2s_wbi_bte = 2'b00;
  
   wb_mem_wrapper
     #(.MEM_SIZE  (BOOTROM_SIZE),
       .INIT_FILE (bootrom_file))
   bootrom
     (.i_clk    (wb_clk),
      .i_rst    (wb_rst),
      .i_wb_adr (wb_m2s_rom_adr[$clog2(BOOTROM_SIZE)-1:2]),
      .i_wb_dat (wb_m2s_rom_dat),
      .i_wb_sel (wb_m2s_rom_sel),
      .i_wb_we  (wb_m2s_rom_we),
      .i_wb_cyc (wb_m2s_rom_cyc),
      .i_wb_stb (wb_m2s_rom_stb),
      .o_wb_rdt (wb_s2m_rom_dat),
      .o_wb_ack (wb_s2m_rom_ack));

   assign wb_s2m_rom_err = 1'b0;
   assign wb_s2m_rom_rty = 1'b0;



    wire i_ram_init_done = 1;
    wire i_ram_init_error = 0;

   swervolf_syscon
     #(.clk_freq_hz (clk_freq_hz))
   syscon
     (.i_clk            (clk),
      .i_rst            (wb_rst),

      .i_gpio           (i_gpio),
      .o_gpio           (o_gpio),
      .o_timer_irq      (timer_irq),
      .o_sw_irq3        (sw_irq3),
      .o_sw_irq4        (sw_irq4),
      .i_ram_init_done  (i_ram_init_done),
      .i_ram_init_error (i_ram_init_error),
      .o_nmi_vec        (nmi_vec),
      .o_nmi_int        (nmi_int),

      .i_wb_adr         (wb_m2s_sys_adr[5:0]),
      .i_wb_dat         (wb_m2s_sys_dat),
      .i_wb_sel         (wb_m2s_sys_sel),
      .i_wb_we          (wb_m2s_sys_we),
      .i_wb_cyc         (wb_m2s_sys_cyc),
      .i_wb_stb         (wb_m2s_sys_stb),
      .o_wb_rdt         (wb_s2m_sys_dat),
      .o_wb_ack         (wb_s2m_sys_ack));

   assign wb_s2m_sys_err = 1'b0;
   assign wb_s2m_sys_rty = 1'b0;


  wire      [31:0]   externalResetVector;
  wire               timerInterrupt;
  wire               softwareInterrupt;
  wire      [31:0]   externalInterruptArray;
  wire               externalInterrupt;
  wire               externalInterruptS;
  wire               debugReset;
  // wire               debug_resetOut;


wb_mem_wrapper
     #(.MEM_SIZE  (RAM_SIZE),
       .INIT_FILE (""))
   ram
     (.i_clk    (clk),
      .i_rst    (~rstn),
      .i_wb_adr (wb_m2s_ram_adr[$clog2(RAM_SIZE)-1:2]),
      .i_wb_dat (wb_m2s_ram_dat),
      .i_wb_sel (wb_m2s_ram_sel),
      .i_wb_we  (wb_m2s_ram_we),
      .i_wb_cyc (wb_m2s_ram_cyc),
      .i_wb_stb (wb_m2s_ram_stb),
      .o_wb_rdt (wb_s2m_ram_dat),
      .o_wb_ack (wb_s2m_ram_ack));

  uart_dpi #(
      .tcp_port(5678),
      .port_name("UART DPI number 1"),
      .welcome_message( "--- Welcome to my UART DPI port ---\n\r" ),
      .UART_DPI_ADDR_WIDTH(10)
    )
    uart_dpi_instance (
      .wb_clk_i	(clk),
      .wb_rst_i	(~rst_n),
      .wb_adr_i	(wb_m2s_uart_adr[11:2]),
      .wb_dat_i	(wb_m2s_uart_dat[31:0]),
      .wb_we_i	(wb_m2s_uart_we),
      .wb_cyc_i	(wb_m2s_uart_cyc),
      .wb_stb_i	(wb_m2s_uart_stb),
      .wb_sel_i	(wb_m2s_uart_sel), // Not used in 8-bit mode
      .wb_dat_o	(wb_s2m_uart_dat),
      .wb_ack_o	(wb_s2m_uart_ack),

      .wb_err_o(),
        // Interrupt request
      .int_o          ( uart_irq )
 );

  wb_ntp ntp_mod(
    .i_clk    (wb_clk),
    .i_rst    (wb_rst),
    .i_wb_adr (wb_m2s_ntp_adr[5:0]),
    .i_wb_dat (wb_m2s_ntp_dat),
    .i_wb_sel (wb_m2s_ntp_sel),
    .i_wb_we  (wb_m2s_ntp_we),
    .i_wb_cyc (wb_m2s_ntp_cyc),
    .i_wb_stb (wb_m2s_ntp_stb),
    .o_wb_rdt (wb_s2m_ntp_dat),
    .o_wb_ack (wb_s2m_ntp_ack));

  VexRiscv core(
    .clk(clk),
    .reset(~rstn),

    //.externalResetVector(32'h80000000),
    .timerInterrupt(timerInterrupt),
    .softwareInterrupt(softwareInterrupt),
    //.externalInterruptArray(externalInterruptArray),
    .externalInterrupt(uart_irq),
    .externalInterruptS(externalInterruptS),

    .iBusWishbone_CYC(              wb_m2s_wbi_cyc),
    .iBusWishbone_STB(              wb_m2s_wbi_stb),
    .iBusWishbone_ACK(              wb_s2m_wbi_ack),
    .iBusWishbone_WE(               wb_m2s_wbi_we),
    .iBusWishbone_ADR(              wb_m2s_wbi_adr[31:2]),
    .iBusWishbone_DAT_MISO(         wb_s2m_wbi_dat),
    .iBusWishbone_DAT_MOSI(         wb_m2s_wbi_dat),
    .iBusWishbone_SEL(              wb_m2s_wbi_sel),
    .iBusWishbone_ERR(              wb_s2m_wbi_err),
    .iBusWishbone_BTE(              wb_m2s_wbi_bte),
    .iBusWishbone_CTI(              wb_m2s_wbi_cti),

    .dBusWishbone_CYC(              wb_m2s_wbd_cyc),
    .dBusWishbone_STB(              wb_m2s_wbd_stb),
    .dBusWishbone_ACK(              wb_s2m_wbd_ack),
    .dBusWishbone_WE(               wb_m2s_wbd_we),
    .dBusWishbone_ADR(              wb_m2s_wbd_adr[31:2]),
    .dBusWishbone_DAT_MISO(         wb_s2m_wbd_dat),
    .dBusWishbone_DAT_MOSI(         wb_m2s_wbd_dat),
    .dBusWishbone_SEL(              wb_m2s_wbd_sel),
    .dBusWishbone_ERR(              wb_s2m_wbd_err),
    .dBusWishbone_BTE(              wb_m2s_wbd_bte),
    .dBusWishbone_CTI(              wb_m2s_wbd_cti),
    
    .jtag_tdo(o_jtag_tdo),
    .jtag_tck(i_jtag_tck),
    .jtag_tms(i_jtag_tms),
    .jtag_tdi(i_jtag_tdi),
    .debugReset(i_jtag_trst_n),
    .debug_resetOut(debug_resetOut)
    


  );

  // assign wb_m2s_wbd_cyc = wb_m2s_wbd_stb | wb_m2s_wbd_we;
  // assign wb_m2s_wbi_cyc = wb_m2s_wbi_stb | wb_m2s_wbi_we;

  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  

endmodule
