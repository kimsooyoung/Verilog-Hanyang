`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: Road Balance 
// Engineer: Kim Soo Young
// 
// Create Date:    03:35:37 04/26/2021 
// Design Name:    alu_32_bit
// Module Name:    alu_32_bit 
// Project Name:   alu_32_bit
// Target Devices: 
// Tool versions: 
// Description: 32bit ALU
//
// Dependencies: 
//
// Revision: 
// Revision 0.01 - File Created
// Additional Comments: 
//
//////////////////////////////////////////////////////////////////////////////////
module alu_32_bit
(
    //----------------------------------------------------------------
    // Input Ports
    //----------------------------------------------------------------
    input	[31:0]                         	a_in, 		//	32 bit first  input data
    input	[31:0]                         	b_in,		//	32 bit second input data
	input	[2:0] 							op,			// operator code
    //----------------------------------------------------------------
    // Output Ports
    //----------------------------------------------------------------
	output	 wire	[31:0]                  result, 	// ALU result
    output   wire 		                   	zero, 		//	carry of two inputs
	output   wire        		            overflow 	//	carry of two inputs
);

// variables for store operation results
wire[31:0] res_and, res_or, res_slt;

// Additional variables for temporal results
// xxx_c_out : stores carry out from each bit operation
// res_xxx 	 : stores bitwise operation results
wire[31:0] add_c_out, sub_c_out;
wire[31:0] res_add, res_sub;

// AND Operation
assign    	res_and 	=	a_in & b_in;

// OR Operation
assign  	res_or 		=	a_in | b_in;

// "add_loop.v" contains actual processes

// ADD Operation
add_loop ALU_Add_Loop(
			.a_in			( a_in		), 	//	one bit input data
			.b_in			( b_in		),	//	one bit input data
			.operation		( op		),	// operation code
			.res_add		( res_add	), 	// ALU result
			.c_out			( add_c_out	)  	//	carry out of two input
);

// SUBTRACT Operation
add_loop ALU_Sub_Loop(
			.a_in			( a_in	 	), 	//	one bit input data
			.b_in			( b_in	 	),	//	one bit input data
			.operation		( op 		),	// operation code
			.res_add		( res_sub 	), 	// ALU result
			.c_out			( sub_c_out )  	//	carry out of two input
);

// SLT Operation
// A < B => 1
// A > B => 0
assign    	res_slt		= (res_sub[31]) ? 32'b1 : 32'b0;
	
// 3 bit MUX
assign 	 	result		= ( op == 3'b000 ) ? res_and :
						  ( op == 3'b001 ) ? res_or  :
						  ( op == 3'b010 ) ? res_add :
						  ( op == 3'b110 ) ? res_sub :
						  ( op == 3'b111 ) ? res_slt :  32'hffff_ffff;
						
// Logical operation for zero bit & overflow bit

// result == 0 => result 1
// result != 0 => result 0
assign 	  zero 			= ~|result;

// There's four cases of overflow
// (+) + (+) => (-)
// (-) + (-) => (+)
// (+) - (-) => (-)
// (-) - (+) => (+)
assign 	  overflow 		= ( op == 3'b010 ) ? (( add_c_out[31] ^ add_c_out[30]) ? 1'b1 : 1'b0):
						  ( op == 3'b110 ) ? (( sub_c_out[31] ^ sub_c_out[30]) ? 1'b1 : 1'b0): 1'b0;
							  
endmodule
