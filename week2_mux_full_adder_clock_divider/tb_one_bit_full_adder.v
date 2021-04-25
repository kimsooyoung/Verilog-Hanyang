`timescale 1ns / 1ps

////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer:
//
// Create Date:   22:39:11 03/29/2021
// Design Name:   one_bit_full_adder
// Module Name:   C:/Users/tge13/Documents/3_1/Computer Architecture/week_2/tb_one_bit_full_adder.v
// Project Name:  week_2
// Target Device:  
// Tool versions:  
// Description: 
//
// Verilog Test Fixture created by ISE for module: one_bit_full_adder
//
// Dependencies:
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
////////////////////////////////////////////////////////////////////////////////

module tb_one_bit_full_adder;

	// Inputs
	reg a;
	reg b;
	reg cin;

	// Outputs
	wire sum;
	wire cout;
	
	integer k;

	// Instantiate the Unit Under Test (UUT)
	one_bit_full_adder uut (
		.a(a), 
		.b(b), 
		.cin(cin), 
		.sum(sum), 
		.cout(cout)
	);
	
	initial begin
		// Initialize Inputs
		a = 0;
		b = 0;
		cin = 0;
		forever
			for(k = 0;k <8 ; k = k + 1)
			begin
				cin = k/4;
				b = (k%4)/2;
				a = k%2;
				#10;
			end
		end
      
endmodule

