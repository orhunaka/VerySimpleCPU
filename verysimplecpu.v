//////////////////////////////////////////////////////////////////////////////////
// Company: Orhun Aka
// Engineer: Orhun Aka
// 
// Create Date:    14:00:00 29/05/2023 
// Design Name: VerySimple_CPU Verilog Module
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
          // ADD Intruction
          if (data_fromRAM[31:28] == 4'b0000)  begin
            addr_toRAM = data_fromRAM[27:14];
            stN = 3'b010; 
          end

          // ADDi Instruction
          if (data_fromRAM[31:28] == 4'b0001)  begin 
            addr_toRAM = data_fromRAM[27:14];
            stN = 3'b010; 
          end

          // NAND Instruction
          if (data_fromRAM[31:28] == 4'b0010)  begin
            addr_toRAM = data_fromRAM[27:14];
            stN = 3'b010; 
          end

          // NANDi Instruction
          if (data_fromRAM[31:28] == 4'b0011)  begin
            addr_toRAM = data_fromRAM[27:14];
            stN = 3'b010; 
          end

          // SRL Instruction
          if (data_fromRAM[31:28] == 4'b0100) begin
            addr_toRAM = data_fromRAM[27:14];
            stN = 3'b010;
          end

          //SRLi Instruction
          if (data_fromRAM[31:28] == 4'b0101) begin
            addr_toRAM = data_fromRAM[27:14];
            stN = 3'b010;
          end

          // LT instruction
          if (data_fromRAM[31:28] == 4'b0110) begin
            addr_toRAM = data_fromRAM[27:14];
            stN = 3'b010;
          end

          // LTi instruction
          if (data_fromRAM[31:28] == 4'b0111) begin
            addr_toRAM = data_fromRAM[27:14];
            stN = 3'b010;
          end

          // MUL instruction
          if (data_fromRAM[31:28] == 4'b1110) begin
            addr_toRAM = data_fromRAM[27:14];
            stN = 3'b010;
          end

          // MULi instruction
          if (data_fromRAM[31:28] == 4'b1111) begin
            addr_toRAM = data_fromRAM[27:14];
            stN = 3'b010;
          end

          // CP Instruction
          if (data_fromRAM[31:28] == 4'b1000) begin
            addr_toRAM = data_fromRAM[13:0];
            stN = 3'b010;
          end

          // CPi Instruction
          if (data_fromRAM[31:28] == 4'b1001) begin
            addr_toRAM = data_fromRAM[27:14];
            stN = 3'b010;
          end

          // CPI Instruction
          if (data_fromRAM[31:28] == 4'b1010) begin
            addr_toRAM = IW[13:0]; // Address of R1
            stN = 3'b010;
          end

          // CPIi Instruction
          if (data_fromRAM[31:28] == 4'b1011) begin
            A = IW[27:14];
            AN = IW[13:0];
            stN = 3'b011;
          end

          // BZJ Intruction
          if (data_fromRAM[31:28] == 4'b1100)  begin
            addr_toRAM = data_fromRAM[27:14];
            stN = 3'b010; 
          end

          // BZJi Instruction
          if (data_fromRAM[31:28] == 4'b1101)  begin 
            addr_toRAM = data_fromRAM[27:14];
            stN = 3'b010; 
          end

        end
        3'b010: begin
          // ADD Instruction
          if (IW[31:28] == 4'b0000) begin
            AN = data_fromRAM;
            addr_toRAM = IW[13:0];
            stN = 3'b011;
          end

          // ADDi Instruction
          if (IW[31:28] == 4'b0001) begin
            addr_toRAM = IW[27:14];
            data_toRAM = data_fromRAM + IW[13:0];
            wrEn = 1'b1;
            PCN = PC + 14'd1;
            if (intr==1'b1 && isr != 1'b1)
              stN = 3'h4;
            else
              stN = 3'h0;
          end

          // NAND instruction
          if (IW[31:28] == 4'b0010) begin
            AN = data_fromRAM;
            addr_toRAM = IW[13:0]; 
            stN = 3'b011;      
          end

          // NANDi instruction
          if (IW[31:28] == 4'b0011) begin
            addr_toRAM = IW[27:14];
            data_toRAM = ~(data_fromRAM & IW[13:0]);
            wrEn = 1'b1;
            PCN = PC + 14'd1;
            if (intr==1'b1 && isr != 1'b1)
              stN = 3'h4;
            else
              stN = 3'h0;
          end

          // SRL instruction
          if (IW[31:28] == 4'b0100) begin
            AN = data_fromRAM;
            addr_toRAM = IW[13:0];
            stN = 3'b011;
          end

          // SRLi instruction
          if (IW[31:28] == 4'b0101) begin
            addr_toRAM = IW[27:14];
            if (IW[13:0] < 32)
              data_toRAM = data_fromRAM >> IW[13:0];
            else
              data_toRAM = data_fromRAM << (IW[13:0] - 32);

            wrEn = 1'b1;
            PCN = PC + 1;
            if (intr==1'b1 && isr != 1'b1)
              stN = 3'h4;
            else
              stN = 3'h0;

          end

          // LT Instruction
          if (IW[31:28] == 4'b0110) begin
            AN = data_fromRAM;
            addr_toRAM = IW[13:0];
            stN = 3'b011;
          end

          // LTi instruction
          if (IW[31:28] == 4'b0111) begin
            addr_toRAM = IW[27:14];
            if (data_fromRAM < IW[13:0])
              data_toRAM = 1;
            else
              data_toRAM = 0;

            wrEn = 1'b1;
            PCN = PC + 1;
            if (intr==1'b1 && isr != 1'b1)
              stN = 3'h4;
            else
              stN = 3'h0;

          end

          // MUL Instruction
          if (IW[31:28] == 4'b1110) begin
            AN = data_fromRAM;
            addr_toRAM = IW[13:0];
            stN = 3'b011;
          end

          // MULi instruction
          if (IW[31:28] == 4'b1111) begin
            addr_toRAM = IW[27:14];
            data_toRAM = data_fromRAM * IW[13:0];

            wrEn = 1'b1;
            PCN = PC + 1;
            if (intr==1'b1 && isr != 1'b1)
              stN = 3'h4;
            else
              stN = 3'h0;

          end

          // CP Instruction
          if (IW[31:28] == 4'b1000) begin
            AN = data_fromRAM;
            addr_toRAM = IW[27:14];
            data_toRAM = AN;
            wrEn = 1'b1;
            PCN = PC + 1;
            if (intr==1'b1 && isr != 1'b1)
              stN = 3'h4;
            else
              stN = 3'h0;

          end

          // CPi Instruction
          if (IW[31:28] == 4'b1001) begin
            addr_toRAM = IW[27:14];
            data_toRAM = IW[13:0];
            wrEn = 1'b1;
            PCN = PC + 1;
            if (intr==1'b1 && isr != 1'b1)
              stN = 3'h4;
            else
              stN = 3'h0;

          end

          // CPI Instruction
          if (IW[31:28] == 4'b1010) begin
            AN = data_fromRAM;
            A = IW[27:14];
            stN = 3'b011;
          end

          // BZJ Instruction
          if (IW[31:28] == 4'b1100) begin
            AN = data_fromRAM;
            addr_toRAM = IW[13:0];
            stN = 3'b011;
          end

          // BZJi instruction
          if (IW[31:28] == 4'b1101) begin
            PCN = data_fromRAM[13:0] + IW[13:0];
            if (intr==1'b1 && isr != 1'b1)
              stN = 3'h4;
            else
              stN = 3'h0;

          end
        end
        3'b011: begin
          // ADD instruction
          if (IW[31:28] == 4'b0000) begin 
            addr_toRAM = IW[27:14]; 
            data_toRAM = A + data_fromRAM;
            wrEn = 1'b1;
            PCN = PC + 14'd1;
            if (intr==1'b1 && isr != 1'b1)  // if there is an interrupt do the ISR
              stN = 3'h4;
            else
              stN = 3'h0;     
          end

          // NAND instruction
          if (IW[31:28] == 4'b0010) begin 
            addr_toRAM = IW[27:14];
            data_toRAM = ~(A & data_fromRAM);
            wrEn = 1'b1;
            PCN = PC + 14'd1;
            if (intr==1'b1 && isr != 1'b1)  // if there is an interrupt do the ISR
              stN = 3'h4;
            else
              stN = 3'h0;     
          end

          // SRL Instruction
          if (IW[31:28] == 4'b0100) begin
            addr_toRAM = IW[27:14];
            if (data_fromRAM < 32)
              data_toRAM = A >> data_fromRAM;
            else
              data_toRAM = A << (data_fromRAM - 32);

            wrEn = 1'b1;
            PCN = PC + 1;
            if (intr==1'b1 && isr != 1'b1)  // if there is an interrupt do the ISR
              stN = 3'h4;
            else
              stN = 3'h0;
          end

          // LT instruction
          if (IW[31:28] == 4'b0110) begin
            addr_toRAM = IW[27:14];
            if (A < data_fromRAM)
              data_toRAM = 1;
            else
              data_toRAM = 0;

            wrEn = 1'b1;
            PCN = PC + 1;
            if (intr==1'b1 && isr != 1'b1)  // if there is an interrupt do the ISR
              stN = 3'h4;
            else
              stN = 3'h0;

          end

          // MUL instruction
          if (IW[31:28] == 4'b1110) begin
            addr_toRAM = IW[27:14];
            data_toRAM = A * data_fromRAM; 
            wrEn = 1'b1;
            PCN = PC + 1;
            if (intr==1'b1 && isr != 1'b1)  // if there is an interrupt do the ISR
              stN = 3'h4;
            else
              stN = 3'h0;

          end

          // CP Instruction
          if (IW[31:28] == 4'b1000) begin
            addr_toRAM = IW[27:14];
            data_toRAM = data_fromRAM;
            wrEn = 1'b1;
            PCN = PC + 1;
            if (intr==1'b1 && isr != 1'b1)  // if there is an interrupt do the ISR
              stN = 3'h4;
            else
              stN = 3'h0;

          end

          // CPI and CPIi instruction
          if (IW[31:28] == 4'b1010 || IW[31:28] == 4'b1011) begin
            addr_toRAM = A; // Address of R1
            data_toRAM = AN; // Value of R2
            wrEn = 1'b1; // Enable write operation
            PCN = PC + 1; // Increment PC
            if (intr==1'b1 && isr != 1'b1)  // if there is an interrupt do the ISR
              stN = 3'h4;
            else
              stN = 3'h0;
          end

          // BZJ instruction
          if (IW[31:28] == 4'b1100) begin
            if (data_fromRAM == 32'd0) 
              PCN = A[13:0];
            else
              PCN = PC +14'd1;

            if (IW[27:14] == 14'h0006) begin //Return from interrupt
              intrN = 1'b0;
              isrN = 1'b0;
            end  
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
    blram.memory[0] = 32'h28014;
    blram.memory[10] = 32'h14;
    blram.memory[20] = 32'h32; 
  end
endmodule