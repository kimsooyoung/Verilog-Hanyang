`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date:    22:32:03 03/29/2021 
// Design Name: 
// Module Name:    one_bit_full_adder 
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
module one_bit_full_adder( a, b, cin, sum, cout );
	input a, b, cin;
	output sum, cout;
	wire s1, c1, c2;
	
	xor g1(s1, a, b);
	and g2(c1, a, b);
	and g3(c2, s1, cin);
	xor g4(sum, s1, cin);
	xor g5(cout, c1, c2);


endmodule
