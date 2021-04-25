`timescale 1ns / 1ps

////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer:
//
// Create Date:   12:05:42 03/30/2021
// Design Name:   four_bit_mux_cond
// Module Name:   C:/Users/tge13/Documents/3_1/Computer Architecture/week_2/tb_four_bit_mux_cond.v
// Project Name:  week_2
// Target Device:  
// Tool versions:  
// Description: 
//
// Verilog Test Fixture created by ISE for module: four_bit_mux_cond
//
// Dependencies:
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
////////////////////////////////////////////////////////////////////////////////

module tb_four_bit_mux_cond;

	// Inputs
	reg [1:0] sel;
	reg [3:0] a;
	reg [3:0] b;
	reg [3:0] c;
	reg [3:0] d;

	// Outputs
	wire [3:0] y;

	// Instantiate the Unit Under Test (UUT)
	four_bit_mux_cond uut (
		.sel(sel), 
		.a(a), 
		.b(b), 
		.c(c), 
		.d(d), 
		.y(y)
	);

	initial begin
		// Initialize Inputs
		a = 4'b0001; 
		b = 4'b0010; 
		c = 4'b0100; 
		d = 4'b1000;
		
		#80
	
		a = 4'b1100; 
		b = 4'b0011; 
		c = 4'b0110; 
		d = 4'b1001;

		// Wait 100 ns for global reset to finish
        
		// Add stimulus here
		
	end
	
	initial sel = 2'b00;
	
	always #20 sel = sel + 1;
      
endmodule

