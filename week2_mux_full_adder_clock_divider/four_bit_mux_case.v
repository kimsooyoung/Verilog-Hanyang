`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date:    11:52:11 03/30/2021 
// Design Name: 
// Module Name:    four_bit_mux_case 
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
module four_bit_mux_case( sel, a, b, c, d, y );
	input [1:0] sel;
	input [3:0] a, b, c, d;
	output reg [3:0] y;
	
	always @(*) begin
		case (sel)
			0: y = a;
			1: y = b;
			2: y = c;
			3: y = d;
			default: 
				y = 4'bx;
		endcase
	end
endmodule
