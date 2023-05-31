//////////////////////////////////////////////////////////////////////////////////
// Company: Orhun Aka
// Engineer: Orhun Aka
// 
// Create Date:    14:00:00 29/05/2023 
// Design Name: VerySimple_CPU Testbench
// Module Name:    vscpu
// Project Name: VerySimpleCPU
// Target Devices: 
// Tool versions: 
// Description: A very simple CPU, that can do some basic addition, multiplication, etc.
//
// Dependencies: ISE Design Suite 14.7
//
// Revision: 
// Revision 0.01 - File Created
// Additional Comments: 
//
//////////////////////////////////////////////////////////////////////////////////

`timescale 1ns / 1ns
module tb;
  parameter SIZE = 14, DEPTH = 2**14;

  reg clk;
  initial begin
    clk = 1;
    forever
      #5 clk = ~clk;
  end

  reg rst;
  initial begin
    rst = 1;
    repeat (10) @(posedge clk);
    rst <= #1 0;
    repeat (300) @(posedge clk);
    $finish;
  end

  reg interrupt;
  initial begin
    interrupt = 1'b0;
    repeat (56) @(negedge clk);
    interrupt <=#1 1;
    @ (posedge clk);
    interrupt <=#1 0;
  end

  wire wrEn;
  wire [SIZE-1:0] addr_toRAM;
  wire [31:0] data_toRAM, data_fromRAM;

  VerySimpleCPU inst_VerySimpleCPU(
    .clk(clk),
    .rst(rst),
    .wrEn(wrEn),
    .data_fromRAM(data_fromRAM),
    .addr_toRAM(addr_toRAM),
    .data_toRAM(data_toRAM),
    .interrupt (interrupt)

  );

  blram #(SIZE, DEPTH) inst_blram(
    .clk(clk),
    .i_we(wrEn),
    .i_addr(addr_toRAM),
    .i_ram_data_in(data_toRAM),
    .o_ram_data_out(data_fromRAM)
  );
  initial begin
    $dumpfile("dump.vcd");
    $dumpvars;
    blram.memory[0] = 32'h28014;
    blram.memory[10] = 32'h14;
    blram.memory[20] = 32'h32;
  end
endmodule