`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date:    14:29:31 04/26/2021 
// Design Name: 
// Module Name:    add_loop 
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
module add_loop(
    //----------------------------------------------------------------
    // Input Ports
    //----------------------------------------------------------------
    input	[31:0]                         	a_in, 		//	32 bit first  input data
    input	[31:0]                         	b_in,		//	32 bit second input data
	input	[2:0] 						   	operation, // operator code
    //----------------------------------------------------------------
    // Output Ports
    //----------------------------------------------------------------
	output	wire [31:0]                 	res_add, 	// ALU Add result
    output  wire [31:0]                   	c_out  		//	carry of two inputs
);

// Initial status setup required
// Set first carry_in

// invert logic for 2's complement rule
// It only happends to B
wire 		is_invert, first_c_in;
assign 		is_invert  = (operation == 3'b110) ? 1'b1 : 1'b0;
assign  	first_c_in = (operation == 3'b110) ? 1'b1 : 1'b0;

// 2's complement rule Preserves last bit even Sign conversion occurs
one_bit_alu u1(
			.a_in		( a_in[0]	 ),
			.b_in		( b_in[0]	 ),
			.b_invert	( is_invert	 ),
			.carry_in	( first_c_in ),
			.less		( 1'b1		 ),
			.op			( 2'b10		 ),
			.result		( res_add[0] ),
			.carry_out	( c_out[0]	 )
);

// Apply bitwise add for all the others bits 
generate
		for (genvar i = 1; i < 32; i = i+1) begin 
				one_bit_alu ALU_Sub(
					 .a_in      ( a_in[i]	 ),
					 .b_in      ( b_in[i]	 ),
					 .b_invert  ( is_invert	 ),
					 .carry_in  ( c_out[i-1] ),
					 .less      ( 1'b1		 ),
					 .op        ( 2'b10		 ),

					 .result    ( res_add[i] ),
					 .carry_out ( c_out[i]	 )
				);
		end
endgenerate

endmodule
