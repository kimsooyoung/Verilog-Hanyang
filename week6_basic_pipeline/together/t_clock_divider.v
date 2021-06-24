`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date:    23:56:55 06/21/2021 
// Design Name: 
// Module Name:    clock_divider 
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
module clock_divider(
	input clk,
	input rst,
	output reg clk_operating,
	output my_clk
    );
	 
	reg [24:0] count;
	
   assign my_clk = count[10];
	
	always @(posedge clk, posedge rst) begin
		if (rst) begin
			count <= 4'b0;
			clk_operating <= 1'b0;
		end
		
		else begin
			count <= count + 1;
			clk_operating <= ((count == 25'b1_1111_1111_1111_1111_1111_1111) ? ~clk_operating: clk_operating );
			//clk_operating <= ((count[3]) ? ~clk_operating: clk_operating );
		end
	end

endmodule


