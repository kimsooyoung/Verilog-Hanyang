`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date:    12:01:03 03/30/2021 
// Design Name: 
// Module Name:    four_bit_mux_cond 
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
module four_bit_mux_cond(sel, a, b, c, d, y);
	input [1:0] sel;
	input [3:0] a, b, c, d;
	output [3:0] y;
	
	assign y = (sel == 0) ? a : 
				  (sel == 1) ? b : 
				  (sel == 1) ? c :
				  (sel == 1) ? d :  4'bx;

endmodule
