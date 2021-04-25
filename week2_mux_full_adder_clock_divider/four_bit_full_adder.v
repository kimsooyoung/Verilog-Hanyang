`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date:    12:13:30 03/30/2021 
// Design Name: 
// Module Name:    four_bit_full_adder 
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
module four_bit_full_adder(
	input [3:0] a,
	input [3:0] b,
	output [3:0] sum,
	output cout
	);
	wire c1, c2, c3;
	
	fulladder_1bit u0(a[0], b[0], 1'b0, sum[0], c1);
	fulladder_1bit u1(a[1], b[1], c1, sum[1], c2);
	fulladder_1bit u2(a[2], b[2], c2, sum[2], c3);
	fulladder_1bit u3(a[3], b[3], c3, sum[3], cout);

endmodule


module fulladder_1bit(
	input a,
	input b,
	input ci,
	output s,
	output c0
	);
	
	// assign {c0, s} = a + b + ci;
	assign c0 = ((a + b + ci) & 2'b10) >> 1;
	assign s  = (a + b + ci) & 1'b1;
	
endmodule 