`timescale 1ns / 1ps

////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer:
//
// Create Date:   22:45:18 06/19/2021
// Design Name:   Data_Memory
// Module Name:   C:/Users/tge13/Documents/3_1/Computer Architecture/basic_pipeline/tb_data_memory.v
// Project Name:  basic_pipeline
// Target Device:  
// Tool versions:  
// Description: 
//
// Verilog Test Fixture created by ISE for module: Data_Memory
//
// Dependencies:
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
////////////////////////////////////////////////////////////////////////////////

module tb_data_memory;

	// Inputs
	reg [7:0] MemAddr;
	reg [31:0] Write_Data;
	reg clk;
	reg reset;
	reg MemRead;
	reg MemWrite;

	// Outputs
	wire [31:0] Read_Data;

	// Instantiate the Unit Under Test (UUT)
	Data_Memory uut (
		.MemAddr(MemAddr), 
		.Write_Data(Write_Data), 
		.Read_Data(Read_Data), 
		.clk(clk), 
		.reset(reset), 
		.MemRead(MemRead), 
		.MemWrite(MemWrite)
	);

	initial begin
		// Initialize Inputs
		MemAddr = 0;
		Write_Data = 0;
		clk = 0;
		reset = 1;
		MemRead = 1;
		MemWrite = 0;

		// Wait 100 ns for global reset to finish
		#100;
		reset = 0;
        
		// Add stimulus here
		MemAddr = 8'b0100_0000;
		#50;
		
		MemAddr = 8'b1000_0000;
		#50;
		
		MemAddr = 8'b0100_0000;
		#50;
		
	end

	always begin
		#1 clk <= ~clk;
	end
      
      
endmodule

