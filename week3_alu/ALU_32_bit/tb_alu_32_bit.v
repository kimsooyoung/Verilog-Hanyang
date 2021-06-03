`timescale 1ns / 1ps

////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer:
//
// Create Date:   13:12:46 04/26/2021
// Design Name:   alu_32_bit
// Module Name:   C:/Users/tge13/Documents/3_1/Computer Architecture/week3_alu/tb_alu_32_bit.v
// Project Name:  week3_alu
// Target Device:  
// Tool versions:  
// Description: 
//
// Verilog Test Fixture created by ISE for module: alu_32_bit
//
// Dependencies:
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
////////////////////////////////////////////////////////////////////////////////

module tb_alu_32_bit;

	// Inputs
	reg [31:0] a_in;
	reg [31:0] b_in;
	reg [2:0] op;

	// Outputs
	wire [31:0] result;
	wire zero;
	wire overflow;

	// Instantiate the Unit Under Test (UUT)
	alu_32_bit uut (
		.a_in(a_in), 
		.b_in(b_in), 
		.op(op), 
		.result(result), 
		.zero(zero), 
		.overflow(overflow)
	);

	initial begin
		// Initialize Inputs
		a_in = 32'h0000_0000;
		b_in = 32'h0000_0000;
		op = 3'b000;

		// Wait 100 ns for global reset to finish
		#100;
		       
		// Add stimulus here
		
		//////////////////////////
		// AND operation check  //
		//////////////////////////		
		op	 	=  3'b000;
		a_in 	= 32'h0000_0000; b_in = 32'hffff_ffff; 
		#20;
		a_in 	= 32'hf0f0_f0f0; b_in = 32'hffff_ffff; 
		#20;

		//////////////////////////
		// OR operation check   //
		//////////////////////////			
		op	  	=  3'b001;		
		a_in 	= 32'hffff_0000; b_in = 32'hffff_ffff;
		#20;
		a_in 	= 32'hf0f0_f0f0; b_in = 32'hffff_0000;
		
		//////////////////////////
		// ADD operation check  //
		//////////////////////////		
		op	 	=  3'b010;		
		a_in 	= 32'h0000_0001;
		b_in 	= 32'h0000_0005;
		#20;
		
		a_in 	= 32'h0000_ffff;
		b_in 	= 32'h0000_0001;
		#20;

		//////////////////////////
		// SUB operation check  //
		//////////////////////////			
		op	  	=  3'b110;		
		a_in 	= 32'h0000_0005;
		b_in 	= 32'h0000_0003;
		#20;
		
		a_in 	= 32'h0000_0003;
		b_in 	= 32'h0000_0005;
		#20;

		//////////////////////////
		// Zero operation check //
		//////////////////////////	
		op	  	=  3'b111;

		// A > B => 0
		a_in 	= 32'h0000_0005;
		b_in 	= 32'h0000_0003;
		#20;

		// A < B => 1		
		a_in 	= 32'h0000_0003;
		b_in	= 32'h0000_0005;
		#20;
		
		//////////////////////////
		// Zero operation check //
		//////////////////////////	
		op	  	=  3'b110;
		a_in  	= 32'h0000_ffff;
		b_in 	= 32'h0000_ffff;
		#20;
		
		
		//////////////////////////
		///   Overflow check   ///
		//////////////////////////
		
		// There's four cases of overflow
		// (+) + (+) => (-)
		// (-) + (-) => (+)
		// (+) - (-) => (-)
		// (-) - (+) => (+)
		
		// add case overflow
		// (+) + (+) => (-)
		op	  	=  3'b010;
		a_in 	= 32'h7fff_ffff;
		b_in 	= 32'h4000_0000;
		#20;
		
		// (-) + (-) => (+)
		a_in 	= 32'hbfff_ffff;
		b_in 	= 32'hbfff_ffff;
		#20;
		
		// sub case overflow
		// (+) - (-) => (-)
		op	  	=  3'b110;
		a_in 	= 32'h7fff_ffff;
		b_in 	= 32'hcfff_ffff;
		#20;
		
		// (-) - (+) => (+)
		a_in 	= 32'hbfff_ffff;
		b_in 	= 32'h4000_0001;
		#20;
	end
      
endmodule

