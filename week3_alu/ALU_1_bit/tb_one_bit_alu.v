`timescale 1ns / 1ps

////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer:
//
// Create Date:   15:46:37 04/21/2020
// Design Name:   ALU_1bit
// Module Name:   C:/Xilinx/14.7/ALU_32/tb_ALU_1bit.v
// Project Name:  ALU_32
// Target Device:  
// Tool versions:  
// Description: 
//
// Verilog Test Fixture created by ISE for module: ALU_1bit
//
// Dependencies:
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
////////////////////////////////////////////////////////////////////////////////

module tb_one_bit_alu;

	// Inputs
	reg a_in;
	reg b_in;
	reg carry_in;
	reg b_invert;
	reg less;
	reg [1:0] op;

	// Outputs
	wire result;
	wire carry_out;

	// Instantiate the Unit Under Test (UUT)
	one_bit_alu uut (
		.a_in(a_in), 
		.b_in(b_in), 
		.carry_in(carry_in), 
		.b_invert(b_invert), 
		.less(less), 
		.op(op), 
		
		.result(result), 
		.carry_out(carry_out)
	);
	
	initial begin
		// Initialize Inputs
		a_in 		= 0;
		b_in 		= 0;
		carry_in 	= 0;
		b_invert 	= 0;
		less 		= 0;
		op 			= 0;
		#50;
		
		//////////////////////////
		// AND operation check ///
		//////////////////////////
		op 			= 2'b00;
		a_in 		= 1; b_in 		= 0;
		#50;
		// Expected result => result : 0 / carry_out : 0
		a_in 		= 1; b_in 		= 1;
		#50;
		// Expected result => result : 1 / carry_out : 1
		
		//////////////////////////
		// OR operation check  ///
		//////////////////////////
		op 			= 2'b01;
		a_in 		= 0; b_in 		= 1;
		#20;
		// Expected result => result : 1 / carry_out : 0
		a_in 		= 0; b_in 		= 0;
		#20;
		// Expected result => result : 0 / carry_out : 0
		
		//////////////////////////
		// ADD operation check ///
		//////////////////////////
		op 			= 2'b10;
		a_in 		= 1; b_in 		= 1; carry_in 		= 0;
		#20; 
		// Expected result => result : 0 / carry_out : 1
		a_in 		= 1; b_in 		= 1; carry_in 		= 1;
		#20;
		// Expected result => result : 1 / carry_out : 1
		
		//////////////////////////
		//Invert operation check//
		//////////////////////////
		b_invert 	= 1;
		#20;
		// Expected result => result : 0 / carry_out : 1

		//////////////////////////
		// Less operation check //
		//////////////////////////
		// SUB operation check
		op 			= 2'b11;
		less	 	= 1'b0;
		#20;
		// Expected result => result : 0 / carry_out : 0
		less 		= 1'b1;
		#20;
		// Expected result => result : 1 / carry_out : 1
	end			
endmodule

