`timescale 1ns / 1ps

////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer:
//
// Create Date:   12:50:07 03/30/2021
// Design Name:   four_bit_full_adder
// Module Name:   C:/Users/tge13/Documents/3_1/Computer Architecture/week_2/tb_four_bit_full_adder.v
// Project Name:  week_2
// Target Device:  
// Tool versions:  
// Description: 
//
// Verilog Test Fixture created by ISE for module: four_bit_full_adder
//
// Dependencies:
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
////////////////////////////////////////////////////////////////////////////////

module tb_four_bit_full_adder;

	// Inputs
	reg [3:0] a;
	reg [3:0] b;

	// Outputs
	wire [3:0] sum;
	wire cout;
	integer k;

	// Instantiate the Unit Under Test (UUT)
	four_bit_full_adder uut (
		.a(a), 
		.b(b), 
		.sum(sum), 
		.cout(cout)
	);

	initial begin
		forever begin
			for(k = 0; k < 16; k = k + 1) begin
				b = k;
				a = k / 2;
				
				# 50;
			end
		end
	end
      
endmodule

