//////////////////////////////////////////////////////////////////////////////////
// Company: Orhun Aka
// Engineer: Orhun Aka
// Create Date:    14:00:00 29/05/2023 
// Design Name: VerySimple_CPU Floor Division Verilog Module
// Module Name:    vscpu
// Project Name: VerySimpleCPU
// Description: A very simple CPU, that can do some basic instructions.
// Dependencies: ISE Design Suite 14.7
// Revision 0.01 - File Created
//////////////////////////////////////////////////////////////////////////////////

`timescale 1ns / 1ps
module VerySimpleCPU(clk, rst, data_fromRAM, wrEn, addr_toRAM, data_toRAM, interrupt);
  input clk, rst, interrupt;
  output reg wrEn;
  input [31:0] data_fromRAM;
  output reg [31:0] data_toRAM;
  output reg [13:0] addr_toRAM;

  reg [2:0] st, stN;
  reg [13:0] PC, PCN;
  reg [31:0] IW, IWN, A, AN;

  reg 	intr, intrN,isr, isrN;

  always @(posedge clk) begin
    st<=stN;
    PC<=PCN;
    A<=AN;
    IW<= IWN;
    intr<= intrN;
    isr <= isrN;
  end

  always @ * begin
    stN= 3'bxxx;
    addr_toRAM = 14'dX;
    data_toRAM = 32'dx;
    AN = 32'dx;
    PCN = PC;
    wrEn = 1'b0;
    IWN = IW;
    intrN= (interrupt && intr==1'b0)? 1'b1: intr;
    isrN = isr;
    if (rst) begin
      stN = 3'b000;
      PCN = 14'd0; 
      intrN= 1'b0;
      isrN = 1'b0;
    end
    else begin    
      case (st)
        3'b000: begin
          // Fetch instruction
          addr_toRAM = PC;
          stN = 3'b001;
        end
        3'b001: begin
          IWN = data_fromRAM;
          // Floor Division Instruction reached from Memory. (4'b0000)
          // First operand is the dividend.
          if (data_fromRAM[31:28] == 4'b0000)  begin
            addr_toRAM = data_fromRAM[27:14];
            stN = 3'b010; 
          end
        end
        3'b010: begin
          // First operand read.
          // Address of the second operand is sent.
          // Second operand is the divider.
          if (IW[31:28] == 4'b0000) begin
            AN = data_fromRAM;
            addr_toRAM = IW[13:0];
            stN = 3'b011;
          end
        end
        3'b011: begin
          // Second operand read.
          // Doing the floor division.
          if (IW[31:28] == 4'b0000) begin 
            addr_toRAM = IW[27:14]; 
            data_toRAM = A / data_fromRAM;
            wrEn = 1'b1;
            PCN = PC + 14'd1;
            if (intr==1'b1 && isr != 1'b1)  // if there is an interrupt do the ISR
              stN = 3'h4;
            else
              stN = 3'h0;     
          end
        end
        3'b100: begin // New state to get the ISR address from @5       
          wrEn = 1'b0;
          addr_toRAM = 32'h5; //Get the ISR address
          stN = 3'h5;
          isrN = 1'b1;         
        end
        3'b101: begin // New state to store the next ınstruction address @6    
          wrEn = 1'b1;
          data_toRAM = PC; 
          addr_toRAM = 32'h6; // Store next PC value @6
          PCN = data_fromRAM[13:0];
          stN = 3'h0;         
        end
      endcase
    end
  end
endmodule

module blram(clk, i_we, i_addr, i_ram_data_in, o_ram_data_out);
  parameter SIZE = 14, DEPTH = 2**14;

  input clk;
  input i_we;
  input [SIZE-1:0] i_addr;
  input [31:0] i_ram_data_in;
  output reg [31:0] o_ram_data_out;

  reg [31:0] memory[0:DEPTH-1];

  always @(posedge clk) begin
    o_ram_data_out <= #1 memory[i_addr[SIZE-1:0]];
    if (i_we)
      memory[i_addr[SIZE-1:0]] <= #1 i_ram_data_in;
  end
  initial begin
    blram.memory[0] = 32'h2800b;
    blram.memory[10] = 32'h64;
    blram.memory[11] = 32'h9;
  end
endmodule