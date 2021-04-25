`timescale 1ns / 1ps

////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer:
//
// Create Date:   19:29:09 03/29/2021
// Design Name:   assignment
// Module Name:   C:/Users/tge13/Documents/3_1/Computer Architecture/print_my_id/tb_assignment.v
// Project Name:  print_my_id
// Target Device:  
// Tool versions:  
// Description: 
//
// Verilog Test Fixture created by ISE for module: assignment
//
// Dependencies:
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
////////////////////////////////////////////////////////////////////////////////

module tb_assignment;

	// Inputs
	reg [31:0] a;

	// Outputs
	wire [31:0] c;

	// Instantiate the Unit Under Test (UUT)
	assignment uut (
		.a(a), 
		.c(c)
	);

	initial begin
		// Initialize Inputs
		a = 0;

		// Wait 100 ns for global reset to finish
		#100;
        
		// Add stimulus here
		a = 2015036580;
	end
      
endmodule

