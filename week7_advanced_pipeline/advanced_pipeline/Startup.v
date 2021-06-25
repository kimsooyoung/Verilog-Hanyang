`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date:    23:36:42 06/21/2021 
// Design Name: 
// Module Name:    startup 
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
module Startup(
	input	clk_50MHz,
	input reset,
	output [15:0] ALU_Result,
	output reg [3:0] digit,
	output reg [7:0] fnd,
	output Branch,
	output Jump,
	output RS_RT_Equal,
	output Branch_Taken
    );
	 
	reg [24:0] counter;
	reg [3:0] number;
	
	reg [15:0] test_led = 0;
	wire clk_slow;
	reg digit_sel;
	
	reg clk_operating;

	assign clk_slow = clk_operating;
	
	wire slow_clock;
	wire [31:0] PC;
	
	clock_divider my_divider(.clk(clk_50MHz), .rst(reset), .clk_operating(slow_clock), .my_clk(clk));
	//assign debug_slow_clock = slow_clock;
	
	advanced_pipeline my_pipeline(.clk(slow_clock), .reset(reset), .result(ALU_Result), .PC_now(PC), 
		.d_Branch(Branch), .d_Jump(Jump), .d_RS_RT_Equal(RS_RT_Equal), .d_Branch_Taken(Branch_Taken));
	//assign Debug_ALU = ALU_Result;
	
	//Fnd number
	// always @(posedge clk or negedge reset)

	always @(posedge clk or posedge reset)
		begin 
			if (reset)
				begin 
					digit_sel<=0;
					digit<=4'b0000;
					number<=0;
				end
			else
				begin
					digit_sel <= digit_sel + 1'b1;
						case(digit_sel)
							0: begin digit<=4'b1110; number<= PC % 32'd10; end
							1: begin digit<=4'b1101; number<= PC / 32'd10; end
						endcase
				end
			end
		//Fnd number
	always @(*)
		case (number)
		4'h0 : fnd=8'b00000011;
		4'h1 : fnd=8'b10011111;
		4'h2 : fnd=8'b00100101;
		4'h3 : fnd=8'b00001101;
		4'h4 : fnd=8'b10011001;
		4'h5 : fnd=8'b01001001;
		4'h6 : fnd=8'b01000001;
		4'h7 : fnd=8'b00011011;
		4'h8 : fnd=8'b00000001;
		4'h9 : fnd=8'b00011001;
		4'ha : fnd=8'b10001001;
		4'hb : fnd=8'b10000011;
		4'hc : fnd=8'b01100011;
		4'hd : fnd=8'b10000101;
		4'he : fnd=8'b01100001;
		4'hf : fnd=8'b01110001;
		default : fnd=8'b00000000;
			endcase
	
	

endmodule

