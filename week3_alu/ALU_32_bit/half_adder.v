//*******************************************************************/
//  $File: half_adder.v
//  $Date: 2016-03-22
//  $Revision: 1.0 $
//  $Author: ENCLab
//  $State: 1.0 - Initial Release
//*******************************************************************/
//  $Change History:
//
//              Revision 1.0 ENCLab
//              Initial Release
//*******************************************************************/
//  $Description
//
//  one-bit half adder
//
//*******************************************************************/

`timescale 1 ns / 1 ps

module half_adder
(
    //----------------------------------------------------------------
    // Input Ports
    //----------------------------------------------------------------
    input                               x_in, 		//	one bit input data
    input                               y_in,		//	one bit input data

    //----------------------------------------------------------------
    // Output Ports
    //----------------------------------------------------------------
    output  wire                        s_out, 		// 	sum of two input
    output  wire                        c_out 		//	carry of two input
);

//----------------------------------------------------------------
// code for operation start
//----------------------------------------------------------------

//  output port
assign  	s_out 	=	x_in ^ y_in;
assign      c_out 	=	x_in & y_in;

// 	you can write the code with gate level modeling
//	xor1 sum 	( s_out, x_in, y_in );
//	and1 carry 	( c_out, x_in, y_in );

//	or like this
//	assign 	{ c_out, s_out } 	=	x_in + y_in;

endmodule
