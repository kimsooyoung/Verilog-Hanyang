`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: Road Balance
// Engineer: Kim Soo Young
// 
// Create Date:    00:45:50 04/26/2021 
// Design Name:    one_bit_alu
// Module Name:    one_bit_alu 
// Project Name:   one_bit_alu
// Target Devices: 
// Tool versions: 
// Description: Basic 1-Bit ALU
//
// Dependencies: 
//
// Revision: 
// Revision 0.01 - File Created
// Additional Comments: This is my own project
//
//////////////////////////////////////////////////////////////////////////////////
module one_bit_alu
(
    //----------------------------------------------------------------
    // Input Ports
    //----------------------------------------------------------------
    input                               a_in, 		//	one bit first  input data
    input                               b_in,		//	one bit second input data
	input 	 							b_invert,  // one bit invert flag for second data
	input 							  	carry_in,  // one bit carry from previous operation
	input 								less,		// one bit less input data
	input[1:0] 							op,			// two bit operator code 
    //----------------------------------------------------------------
    // Output Ports
    //----------------------------------------------------------------
	output  wire                        result,		// ALU result
    output  wire                        carry_out 	//	carry of two inputs
);

// inverter for b_in
assign     b_inv  	= ( b_invert == 1'b1 ) ? ~b_in : b_in;

// bitwise AND and OR operation result
assign     res_and 	=	a_in & b_in;
assign     res_or 	=	a_in | b_in;

// temporal variable for Add operation
wire       res_sum;

// Execute full adder with carry_in
full_adder ALU_half_adder (
			 .x_in        ( a_in          ),
			 .y_in        ( b_inv         ),
			 .c_in        ( carry_in      ),
			 .s_out       ( res_sum       ),
			 .c_out       ( carry_out     )
);

// 2 bit MUX for operation selection
assign 	   result 	= ( op == 2'b00 ) ? res_and : 
					  ( op == 2'b01 ) ? res_or  : 
					  ( op == 2'b10 ) ? res_sum :
					  ( op == 2'b11 ) ? less    :  1'b0;

endmodule
