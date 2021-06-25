`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date:    18:19:35 05/17/2021 
// Design Name: 
// Module Name:    assignment 
// Project Name: 
// Target Devices: 
// Tool versions: 
// Description: 
//
// Dependencies: 
//
// Revision: 
// Revision 0.01 - File Created
// Additional Comments: 
//
//////////////////////////////////////////////////////////////////////////////////
module Data_Memory(
		input [8:0]MemAddr,
		input MemRead,
		input MemWrite,
		input [31:0]Write_Data,
		output reg [31:0]Read_Data
    );
	 
	 reg [31:0] memory[127:0];
	 
	 
	 always @(*) begin
		if (MemRead && !MemWrite) begin
			Read_Data <= memory[MemAddr[8:2]];
		end

		else if (MemWrite && !MemRead) begin
			memory[MemAddr[8:2]] <= Write_Data;
		end
		
		else begin
			Read_Data <= 32'bx;
		end
	 end
		 
endmodule

