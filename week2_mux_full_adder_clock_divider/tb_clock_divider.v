`timescale 1ns / 1ps

////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer:
//
// Create Date:   13:39:00 03/30/2021
// Design Name:   clock_divider
// Module Name:   C:/Users/tge13/Documents/3_1/Computer Architecture/week_2/tb_clock_divider.v
// Project Name:  week_2
// Target Device:  
// Tool versions:  
// Description: 
//
// Verilog Test Fixture created by ISE for module: clock_divider
//
// Dependencies:
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
////////////////////////////////////////////////////////////////////////////////

module tb_clock_divider;

	// Inputs
	reg clk;
	reg rst;

	// Outputs
	wire clk_operating;

	// Instantiate the Unit Under Test (UUT)
	clock_divider uut (
		.clk(clk), 
		.rst(rst), 
		.clk_operating(clk_operating)
	);

	initial begin
		// Initialize Inputs
		clk <= 0;
		rst <= 1;

		// Wait 100 ns for global reset to finish
		#100;
		rst <= 0;
	end
	
	always begin
		#1 clk <= ~clk;
	end
      
endmodule

