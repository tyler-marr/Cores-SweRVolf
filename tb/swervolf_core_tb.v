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
   output wire o_debug_resetOut,

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
   wire  o_uart_tx;

   uart_decoder #(115200) uart_decoder (o_uart_tx);

`endif
//`define RAMSIZE 32'hBFFFFFFF
`define RAMSIZE 32'h7FFFFFFF
`define ram_start_address 32'h80000000
`define real_to_ram(c) ((c - `ram_start_address)/8)
   reg [1023:0] ram_init_file;


   initial begin
      if (|$test$plusargs("jtag_vpi_enable"))
          $display("JTAG VPI enabled. Not loading RAM");
      else if ($value$plusargs("ram_init_file=%s", ram_init_file)) begin
          $display("Loading RAM contents from %0s", ram_init_file);
          $readmemh(ram_init_file, vex.ram.ram.mem);          
      end else begin 
          $readmemh("/scratch/wfhome/wand/swervolf/fusesoc_libraries/vex_soc/sw/emu.vh",
              vex.ram.ram.mem);

          $readmemh("/scratch/wfhome/wand/swervolf/fusesoc_libraries/vex_soc/sw/linux.vh",
              vex.ram.ram.mem, `real_to_ram(32'hc0000000));
          //VMLINUX=$BUILDROOT/output/images/Image


          $readmemh("/scratch/wfhome/wand/swervolf/fusesoc_libraries/vex_soc/sw/rootfs.vh",
              vex.ram.ram.mem, `real_to_ram(32'hc2000000));
          //RAMDISK=$BUILDROOT/output/images/rootfs.cpio

          $readmemh("/scratch/wfhome/wand/swervolf/fusesoc_libraries/vex_soc/sw/dtb.vh",
              vex.ram.ram.mem, `real_to_ram(32'hc3000000));
          //DTB=$BUILDROOT/board/spinal/vexriscv_sim/rv32.dtb 

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
        .RAM_SIZE(`RAMSIZE),
       .clk_freq_hz (32'd50_000_000),
       .MAX_CYCLES(MAX_CYCLES))
   vex
     (.clk  (clk),
      .rstn (!rst),

      .i_gpio              (64'd0),
      .o_gpio              (gpio_out),
          
    .o_jtag_tdo(o_jtag_tdo),
    .i_jtag_tck(i_jtag_tck),
    .i_jtag_tms(i_jtag_tms),
    .i_jtag_tdi(i_jtag_tdi),
    .i_jtag_trst_n(i_jtag_trst_n),
    .debug_resetOut(o_debug_resetOut)

      );

    // always @(posedge core_clk) begin
    //     wb_valid[1:0]  <= '{rvtop.swerv.dec.dec_i1_wen_wb, rvtop.swerv.dec.dec_i0_wen_wb};
    //     wb_dest[1:0]   <= '{rvtop.swerv.dec.dec_i1_waddr_wb, rvtop.swerv.dec.dec_i0_waddr_wb};
    //     wb_data[1:0]   <= '{rvtop.swerv.dec.dec_i1_wdata_wb, rvtop.swerv.dec.dec_i0_wdata_wb};
    //     if (trace_rv_i_valid_ip !== 0) begin
    //        $fwrite(tp,"%b,%h,%h,%0h,%0h,3,%b,%h,%h,%b\n", trace_rv_i_valid_ip, trace_rv_i_address_ip[63:32], trace_rv_i_address_ip[31:0],
    //               trace_rv_i_insn_ip[63:32], trace_rv_i_insn_ip[31:0],trace_rv_i_exception_ip,trace_rv_i_ecause_ip,
    //               trace_rv_i_tval_ip,trace_rv_i_interrupt_ip);
    //        // Basic trace - no exception register updates
    //        // #1 0 ee000000 b0201073 c 0b02       00000000
    //        for (int i=0; i<2; i++)
    //            if (trace_rv_i_valid_ip[i]==1) begin
    //                commit_count++;
    //                $fwrite (el, "%10d : %6s 0 %h %h %s\n", cycleCnt, $sformatf("#%0d",commit_count),
    //                       trace_rv_i_address_ip[31+i*32 -:32], trace_rv_i_insn_ip[31+i*32-:32],
    //                       (wb_dest[i] !=0 && wb_valid[i]) ?  $sformatf("r%0d=%h", wb_dest[i], wb_data[i]) : "");
    //            end
    //     end
    // end

endmodule
