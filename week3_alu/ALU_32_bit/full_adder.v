//*******************************************************************/
//  $File: full_adder.v
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
//  one-bit full adder
//
//*******************************************************************/

`timescale 1 ns / 1 ps

module full_adder
(
    //----------------------------------------------------------------
    // Input Ports
    //----------------------------------------------------------------
    input                               x_in,       //  one bit input data
    input                               y_in,       //  one bit input data
    input                               c_in,       //  one bit carry data

    //----------------------------------------------------------------
    // Output Ports
    //----------------------------------------------------------------
    output  wire                        s_out,      //  sum of two input
    output  wire                        c_out       //  carry of two input
);

//----------------------------------------------------------------
// internal variables declaration
//----------------------------------------------------------------
wire    wire_sum0;
wire    wire_carry0;
wire    wire_carry1;

//----------------------------------------------------------------
// submodule instantiation
//----------------------------------------------------------------
half_adder U0_half_adder (
    .x_in       ( x_in          ),
    .y_in       ( y_in          ),
    .s_out      ( wire_sum0     ),
    .c_out      ( wire_carry0   )
);

half_adder U1_half_adder (
    .x_in       ( wire_sum0     ),
    .y_in       ( c_in          ),
    .s_out      ( s_out         ),
    .c_out      ( wire_carry1   )
);

//----------------------------------------------------------------
// code for operation start
//----------------------------------------------------------------

//  output port
assign  c_out   =   ( wire_carry0 | wire_carry1 );

endmodule
