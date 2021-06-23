`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date:    12:48:03 06/22/2021 
// Design Name: 
// Module Name:    advanced_pipeline 
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

module advanced_pipeline (clk, reset, result, PC_now, Instruction_now,
	Rs_now, Rt_now, ALU_input_1, ALU_input_2, immi_Shifted, PC_4,
	load_data, Beq_address, Write_Register,
	debug_flag_2, Debug_RegWrite, Debug_Write_Data,
	Debug_ForWardA, Debug_ForWardB);

	input clk, reset;
	output [31:0] result, load_data; // ALU result, Data_memory_read
	output [31:0] PC_now; // current PC value Debugging,  
	output [31:0] Instruction_now; // Fetch stage instruction 
	output [31:0] Rs_now, Rt_now; // output from Register File 
	output [31:0] ALU_input_1, ALU_input_2; // ALU inputs
	output [31:0] PC_4, Beq_address, immi_Shifted; // for Branch Address Debugging
	output [4:0] Write_Register; // Rd value
	output [31:0] debug_flag_2, Debug_Write_Data; // ALU_result in MEM stage, Write_Data for Register File
	output Debug_RegWrite; // RegWrite Op Flag

	output [1:0] Debug_ForWardA, Debug_ForWardB;


	// IF stage
	wire [31:0] PC_in;
	wire [31:0] PC_out;
	wire [31:0] PC_plus4;
	wire [31:0] IF_instruction;

	// ID stage
	wire [31:0] ID_PC_plus4;
	wire [31:0] ID_instruction;
	wire [31:0] Read_data_1, Read_data_2;
	wire [31:0] sign_extended_immi;
	wire ALUSrc, RegWrite;
	wire RegDst, Jump, Branch;
	wire MemRead, MemtoReg, MemWrite;
	wire [1:0] ALUOp;

	// EX stage
	wire [31:0] EX_PC_plus4;
	wire [31:0] EX_instruction;
	wire [31:0] EX_Read_data_1, EX_Read_data_2;
	wire [31:0] EX_sign_extended_immi;
	wire EX_ALUSrc, EX_RegWrite;
	wire EX_RegDst, EX_Jump, EX_Branch;
	wire EX_MemRead, EX_MemtoReg, EX_MemWrite;
	wire [1:0] EX_ALUOp;
	wire [4:0] EX_Write_Register;
	wire [3:0] operation_code;

	wire [31:0] Branch_addr;
	wire [31:0] Shifted_immi;
	wire [31:0] ALU_input_B;
	wire [31:0] ALU_result;
	wire ALU_zero;

	wire [1:0] ForwardA, ForwardB;
	wire [31:0] muxA_out, muxB_out;


	// MEM stage
	wire [31:0] MEM_ALU_result, MEM_Read_data_2;
	wire [4:0] MEM_Write_Register;
	wire [31:0] Data_memory_read;
	wire [31:0] MEM_Branch_addr;
	wire MEM_RegWrite, MEM_MemtoReg, MEM_Branch;
	wire MEM_MemRead, MEM_MemWrite, MEM_Jump;
	wire MEM_ALU_zero;
	wire PCSrc;

	// WB stage
	wire [31:0] WB_Data_memory_read, WB_ALU_result;
	wire [4:0] WB_Write_Register;
	wire [31:0] WB_Write_Data;
	wire WB_RegWrite, WB_MemtoReg;

	//////////////////////////////////
	//// Instruction Fetch stage /////
	//////////////////////////////////
	N_bit_MUX #(32) pc_mux (.input0(PC_plus4), .input1(MEM_Branch_addr), 
	 .mux_out(PC_in), .control(PCSrc));
	Program_Counter program_counter (.clk(clk), .reset(reset),
	 .PC_in(PC_in), .PC_out(PC_out));
	Instruction_Memory instruction_memory (.address(PC_out),
	 .instruction(IF_instruction), .reset(reset));
	ALU_add_only pc_add_4 (.input1(PC_out), .input2(32'b0100), .add_out(PC_plus4));
	IF_ID_Stage_Reg IF_ID_Stage_Unit (.clk(clk), .reset(reset),
	 .PC_plus4_in(PC_plus4), .PC_plus4_out(ID_PC_plus4),
	 .instruction_in(IF_instruction), .instruction_out(ID_instruction));
	
	assign PC_now = PC_out;
	assign Instruction_now = IF_instruction;

	//////////////////////////////////
	//// Instruction Decode stage ////
	//////////////////////////////////
	assign Debug_RegWrite = WB_RegWrite;
	assign Debug_Write_Data = WB_Write_Data;

	Register_File regfile_Unit (.clk(clk), .reset(reset),
	 .Read_Register_1(ID_instruction[25:21]), 
	 .Read_Register_2(ID_instruction[20:16]), 
	 .Write_Register(WB_Write_Register), .Write_Data(WB_Write_Data),
	 .Read_Data_1(Read_data_1), .Read_Data_2(Read_data_2), .RegWrite(WB_RegWrite));
	Sign_Extension immi_sign_extension (.input_16(ID_instruction[15:0]), .output_32(sign_extended_immi));
	Control control_unit (.OpCode(ID_instruction[31:26]),
	 .RegDst(RegDst), .Jump(Jump), .Branch(Branch), 
	 .MemRead(MemRead), .MemtoReg(MemtoReg), .ALUOp(ALUOp), 
	 .MemWrite(MemWrite), .ALUSrc(ALUSrc), .RegWrite(RegWrite));
	ID_EX_Stage_Reg ID_EX_Stage_Unit (.clk(clk), .reset(reset),
	 .RegWrite_in(RegWrite), .RegWrite_out(EX_RegWrite),
	 .MemtoReg_in(MemtoReg), .MemtoReg_out(EX_MemtoReg),
	 .Branch_in(Branch), .Branch_out(EX_Branch),
	 .MemRead_in(MemRead), .MemRead_out(EX_MemRead),
	 .MemWrite_in(MemWrite), .MemWrite_out(EX_MemWrite), 
	 .Jump_in(Jump), .Jump_out(EX_Jump),   
	 .RegDst_in(RegDst), .RegDst_out(EX_RegDst),
	 .ALUSrc_in(ALUSrc), .ALUSrc_out(EX_ALUSrc), 
	 .ALUOp_in(ALUOp), .ALUOp_out(EX_ALUOp), 
	 .PC_plus4_in(ID_PC_plus4), .PC_plus4_out(EX_PC_plus4),
	 .read_data_1_in(Read_data_1), .read_data_1_out(EX_Read_data_1),
	 .read_data_2_in(Read_data_2), .read_data_2_out(EX_Read_data_2), 
	 .sign_extended_immi_in(sign_extended_immi), .sign_extended_immi_out(EX_sign_extended_immi), 
	 .instruction_in(ID_instruction), .instruction_out(EX_instruction));

	assign Rs_now = Read_data_1;
	assign Rt_now = Read_data_2;

	//////////////////////////////////
	///////   Execute  stage  ////////
	//////////////////////////////////
	N_bit_MUX #(5) write_reg_mux (.input0(EX_instruction[20:16]), .input1(EX_instruction[15:11]), 
	 .mux_out(EX_Write_Register), .control(EX_RegDst));
	ALU_Control alu_control_unit (.ALUOp(EX_ALUOp), .f_code(EX_instruction[5:0]), .operation_code(operation_code));

	Forwarding_Unit forwarding_unit (.EX_RegisterRs(EX_instruction[25:21]), .EX_RegisterRt(EX_instruction[20:16]), 
	.MEM_RegisterRd(MEM_Write_Register), .WB_RegisterRd(WB_Write_Register), 
	.EX_MEM_RegWrite(MEM_RegWrite), .MEM_WB_RegWrite(WB_RegWrite),
	.ForwardA(ForwardA), .ForwardB(ForwardB));
	Mux_32bit_3to1 ALU_MuxA (.in00(EX_Read_data_1), .in01(WB_Write_Data), .in10(MEM_ALU_result), 
		.mux_out(muxA_out), .control(ForwardA));
	Mux_32bit_3to1 ALU_MuxB (.in00(EX_Read_data_2), .in01(WB_Write_Data), .in10(MEM_ALU_result), 
		.mux_out(muxB_out), .control(ForwardB));

	assign Debug_ForWardA = ForwardA;
	assign Debug_ForWardB = ForwardB;

	N_bit_MUX #(32) alu_input_mux (.input0(muxB_out), .input1(EX_sign_extended_immi), 
	 .mux_out(ALU_input_B), .control(EX_ALUSrc));
	ALU alu_unit (.input1(muxA_out), .input2(ALU_input_B), 
	 .alu_out(ALU_result), .zero(ALU_zero), .control(operation_code));	
	assign Shifted_immi = { EX_sign_extended_immi[29:0], 2'b00 };
	ALU_add_only alu_add_only_unit (.input1(EX_PC_plus4), .input2(Shifted_immi), .add_out(Branch_addr)); 
	EX_MEM_Stage_Reg EX_MEM_Stage_Unit ( .clk(clk), .reset(reset),
	.RegWrite_in(EX_RegWrite), .RegWrite_out(MEM_RegWrite),
	.MemtoReg_in(EX_MemtoReg), .MemtoReg_out(MEM_MemtoReg),
	.Branch_in(EX_Branch), .Branch_out(MEM_Branch),
	.MemRead_in(EX_MemRead), .MemRead_out(MEM_MemRead),
	.MemWrite_in(EX_MemWrite),.MemWrite_out(MEM_MemWrite),
	.Jump_in(EX_Jump), .Jump_out(MEM_Jump),
	.Branch_addr_in(Branch_addr), .Branch_addr_out(MEM_Branch_addr),
	.ALU_zero_in(ALU_zero), .ALU_zero_out(MEM_ALU_zero),
	.ALU_result_in(ALU_result), .ALU_result_out(MEM_ALU_result),
	.Read_data_2_in(EX_Read_data_2), .Read_data_2_out(MEM_Read_data_2), 
	.RegisterRd_in(EX_Write_Register), .RegisterRd_out(MEM_Write_Register));

	assign PC_4 = EX_PC_plus4;
	assign immi_Shifted = Shifted_immi;
	assign ALU_input_1 = muxA_out;
	assign ALU_input_2 = ALU_input_B;
	assign Beq_address = Branch_addr;
	
	assign result = ALU_result;

	//////////////////////////////////
	///////   Memory  stage  /////////
	//////////////////////////////////
	Data_Memory data_memory_unit (.clk(clk), .reset(reset), .MemAddr(MEM_ALU_result[7:0]), 
	 .Write_Data(MEM_Read_data_2), .Read_Data(Data_memory_read), 
	 .MemRead(MEM_MemRead), .MemWrite(MEM_MemWrite));
	and (PCSrc, MEM_Branch, MEM_ALU_zero);

	assign debug_flag_2 = MEM_ALU_result;
	assign load_data = Data_memory_read;

	MEM_WB_Stage_Reg MEM_WB_Stage_Unit (.clk(clk), .reset(reset),
	.RegWrite_in(MEM_RegWrite), .RegWrite_out(WB_RegWrite), 
	.MemtoReg_in(MEM_MemtoReg), .MemtoReg_out(WB_MemtoReg), 
	.Data_memory_read_in(Data_memory_read), .Data_memory_read_out(WB_Data_memory_read), 
	.ALU_result_in(MEM_ALU_result), .ALU_result_out(WB_ALU_result),
	.Write_Register_in(MEM_Write_Register), .Write_Register_out(WB_Write_Register));
	
	//////////////////////////////////
	///   Write Back stage      //////
	//////////////////////////////////
	N_bit_MUX #(32) write_data_mux (.input0(WB_ALU_result), .input1(WB_Data_memory_read), 
	 .mux_out(WB_Write_Data), .control(WB_MemtoReg));

	assign Write_Register = WB_Write_Register;
endmodule

// IF/ID stage register
module IF_ID_Stage_Reg (clk, reset, PC_plus4_in, PC_plus4_out, 
	instruction_in, instruction_out);

	input clk, reset;
	input [31:0] PC_plus4_in, instruction_in;

	output reg [31:0] PC_plus4_out, instruction_out;

	always @(posedge clk or negedge reset) begin

		if (!reset) begin
			PC_plus4_out <= 32'b0;
			instruction_out <= 32'b0;
		end

		else begin
			PC_plus4_out <= PC_plus4_in;
			instruction_out <= instruction_in;
		end
	end
	
endmodule


// ID/EX stage register
module ID_EX_Stage_Reg (clk, reset, RegWrite_in, RegWrite_out, MemtoReg_in, MemtoReg_out,
	Branch_in, Branch_out, MemRead_in, MemRead_out, MemWrite_in, MemWrite_out,
	Jump_in, Jump_out, RegDst_in, RegDst_out, ALUSrc_in, ALUSrc_out, ALUOp_in, ALUOp_out,
	PC_plus4_in, PC_plus4_out, read_data_1_in, read_data_1_out, 
	read_data_2_in, read_data_2_out, 
	sign_extended_immi_in, sign_extended_immi_out,
	instruction_in, instruction_out);

	// WB control signal
	input RegWrite_in, MemtoReg_in;
	output reg RegWrite_out, MemtoReg_out;
	// MEM control signal
	input Branch_in, MemRead_in, MemWrite_in, Jump_in;
	output reg Branch_out, MemRead_out, MemWrite_out, Jump_out;
	// EX control signal
	input RegDst_in, ALUSrc_in;
	output reg RegDst_out, ALUSrc_out;
	input [1:0] ALUOp_in;
	output reg [1:0] ALUOp_out;
	// addr content
	input [31:0] PC_plus4_in;
	output reg [31:0] PC_plus4_out;
	// data content
	input [31:0] read_data_1_in, read_data_2_in, sign_extended_immi_in;
	output reg [31:0] read_data_1_out, read_data_2_out, sign_extended_immi_out;
	// reg content
	input [31:0] instruction_in;
	output reg [31:0] instruction_out;
	// general signal
	input clk, reset;
	
	always @(posedge clk or negedge reset) begin
		if (!reset) begin
			RegWrite_out = 1'b0; MemtoReg_out = 1'b0;
			Branch_out = 1'b0; MemRead_out = 1'b0;
			MemWrite_out = 1'b0; Jump_out = 1'b0;
			RegDst_out = 1'b0; ALUSrc_out = 1'b0;
			ALUOp_out = 2'b0;

			PC_plus4_out = 32'b0; 
			read_data_1_out = 32'b0; read_data_2_out = 32'b0; 
			sign_extended_immi_out = 32'b0;
			instruction_out = 32'b0;	
		end

		else begin
			RegWrite_out = RegWrite_in; MemtoReg_out = MemtoReg_in;
			Branch_out = Branch_in; MemRead_out = MemRead_in;
			MemWrite_out = MemWrite_in; Jump_out = Jump_in;
			RegDst_out = RegDst_in; ALUSrc_out = ALUSrc_in;
			ALUOp_out = ALUOp_in; PC_plus4_out = PC_plus4_in;
			sign_extended_immi_out = sign_extended_immi_in;
			read_data_1_out = read_data_1_in; 
			read_data_2_out = read_data_2_in;
			instruction_out = instruction_in;
		end	
	end	
endmodule

// EX/MEM stage register
module EX_MEM_Stage_Reg (clk, reset,
	RegWrite_in, RegWrite_out, MemtoReg_in, MemtoReg_out, 
	Branch_in, Branch_out, MemRead_in, MemRead_out, 
	MemWrite_in, MemWrite_out, Jump_in, Jump_out,
	Branch_addr_in, Branch_addr_out, ALU_zero_in, ALU_zero_out, 
	ALU_result_in, ALU_result_out, Read_data_2_in, Read_data_2_out, 
	RegisterRd_in, RegisterRd_out);

	// WB control signal
	input RegWrite_in, MemtoReg_in;
	output reg RegWrite_out, MemtoReg_out;
	// MEM control signal
	input Branch_in, MemRead_in, MemWrite_in, Jump_in;
	output reg Branch_out, MemRead_out, MemWrite_out, Jump_out;
	// addr content
	input [31:0] Branch_addr_in;
	output reg [31:0] Branch_addr_out;
	// data content
	input ALU_zero_in;
	output reg ALU_zero_out;
	// results
	input [31:0] ALU_result_in, Read_data_2_in;
	output reg [31:0] ALU_result_out, Read_data_2_out;
	// registers
	input [4:0] RegisterRd_in;
	output reg [4:0] RegisterRd_out;
	// general signal
	input clk, reset;

	always @(posedge clk or negedge reset) begin
		if (!reset) begin
		  RegWrite_out <= 1'b0; MemtoReg_out <= 1'b0;
		  Branch_out <= 1'b0; MemRead_out <= 1'b0;
		  MemWrite_out <= 1'b0; Jump_out <= 1'b0;
		  Branch_addr_out <= 32'b0; ALU_zero_out <= 1'b0;
		  ALU_result_out <= 32'b0; Read_data_2_out <= 32'b0;
		  RegisterRd_out <= 5'b0; 
		end

		else begin
		  RegWrite_out <= RegWrite_in; MemtoReg_out <= MemtoReg_in;
		  Branch_out <= Branch_in; MemRead_out <= MemRead_in;
		  MemWrite_out <= MemWrite_in; Jump_out <= Jump_in;
		  Branch_addr_out <= Branch_addr_in; ALU_zero_out <= ALU_zero_in;
		  ALU_result_out <= ALU_result_in; Read_data_2_out <= Read_data_2_in;
		  RegisterRd_out <= RegisterRd_in;
		end

	end

endmodule

// MEM/WB stage register
module MEM_WB_Stage_Reg (RegWrite_in, RegWrite_out,
	MemtoReg_in, MemtoReg_out, 
	Data_memory_read_in, Data_memory_read_out,
	ALU_result_in, ALU_result_out,
	Write_Register_in, Write_Register_out, clk, reset);
	
	// WB control signal
	input RegWrite_in, MemtoReg_in;
	output reg RegWrite_out, MemtoReg_out;
	// data content
	input [31:0] Data_memory_read_in, ALU_result_in;
	output reg [31:0] Data_memory_read_out, ALU_result_out;
	input [4:0] Write_Register_in;
	output reg [4:0] Write_Register_out;
	// general signal
	input clk, reset;
	
	always @(posedge clk or negedge reset) begin
		if (!reset) begin
			RegWrite_out <= 1'b0; MemtoReg_out <= 1'b0;
			Data_memory_read_out <= 32'b0;  ALU_result_out <= 32'b0;
			Write_Register_out <= 5'b0;
		end

		else begin
			RegWrite_out <= RegWrite_in; MemtoReg_out <= MemtoReg_in;
			Data_memory_read_out <= Data_memory_read_in; ALU_result_out <= ALU_result_in;
			Write_Register_out <= Write_Register_in;
		end
	end
endmodule

// PC
module Program_Counter (clk, reset, PC_in, PC_out);

	input clk, reset;
	input [31:0] PC_in;

	output reg [31:0] PC_out;

	always @ (posedge clk or negedge reset)	begin
		if (!reset)
			PC_out <= 0;
		else
			PC_out <= PC_in;
	end
endmodule

// contains hard-code instructions
module Instruction_Memory (address, instruction, reset);
	input reset;
	input [31:0] address;
	output [31:0] instruction;

	reg [31:0] mem [7:0]; // 8 instructions
	integer k;
	
	// get instruction right away
	assign instruction = mem[address[6:2]];

	// Initial setup at reset posedge
	always @(negedge reset) begin
		for (k = 0; k < 8; k = k + 1) begin
			mem[k] = 32'b0; // add $0 $0 $0
		end

		mem[0] = 32'b100011_00010_00001_0000000000000100; // lw $1, 4($2)
		mem[1] = 32'b000000_00001_00101_00100_00000_100010; // sub $4, $1, $5
		mem[2] = 32'b000000_00001_00111_00110_00000_100100; // and $6, $1, $7
		mem[3] = 32'b000000_00001_01001_01000_00000_100101; // or $8, $1, $9
		mem[5] = 32'b000100_00110_00000_1111111111111011; // beq $6, $0, Label (-5)
		
		// mem[0] = 32'b000000_00011_00100_00010_00000_100000; // add $2, $3, $4
		// mem[1] = 32'b000000_00011_00100_00001_00000_100010; // sub $1, $3, $4
		// mem[2] = 32'b100011_00110_00101_0000000000000000; // lw $5, 0($6)
		// mem[3] = 32'b000100_00011_00100_1111111111111100; // beq $3, $4, Label (-4)
	end
endmodule

// 32-bit ALU for addition only
module ALU_add_only (input1, input2, add_out);
	input [31:0] input1, input2;
	output [31:0] add_out;

	assign add_out=input1+input2;
endmodule

// N_bit_MUX for Usability
module N_bit_MUX (input0, input1, mux_out, control);
	parameter N = 32;
	
	input [N-1:0] input0, input1;
	input control;
	output [N-1:0] mux_out;

	assign mux_out = control ? input1 : input0;
endmodule

// sync register file (write/read occupy half cycle each)
// write: on rising edge; data width 32 bit; address width 5 bit
// read: on falling edge; data width 32 bit; address width 5 bit
module Register_File (Read_Register_1, Read_Register_2, 
	Write_Register, Write_Data, Read_Data_1, Read_Data_2,
	RegWrite, clk, reset);

	input [4:0] Read_Register_1, Read_Register_2, Write_Register;
	input [31:0] Write_Data;
	input clk, reset, RegWrite;
	output reg [31:0] Read_Data_1, Read_Data_2;


	reg [31:0] mem [31:0];
	integer k;
 	
	always @(posedge clk or negedge reset) begin
		if (!reset) begin
			for (k = 0; k < 32; k = k + 1) begin
				mem[k] <= 32'b0;
			end
			mem[1] <= 20;
			mem[2] <= 8;
			mem[5] <= 2;
			mem[6] <= 0;
			mem[7] <= 1;
			mem[9] <= 3;
			// mem[3] = 32'b0011;
			// mem[4] = 32'b0011;
			// mem[6] = 32'h0000_0040;
		end
		
		else if (RegWrite) begin
			mem[Write_Register] <= Write_Data;
		end

	end

	always @(negedge clk) begin
		Read_Data_1 <= mem[Read_Register_1];
		Read_Data_2 <= mem[Read_Register_2];
	end
endmodule

// sign-extend the 16-bit input to the 32_bit output
module Sign_Extension (input_16, output_32);
	input [15:0] input_16;
	output [31:0] output_32;
	
	assign output_32[15:0]  = input_16[15:0];
	assign output_32[31:16] = input_16[15] ? 16'b1111_1111_1111_1111: 16'b0;
endmodule

// Control Path
module Control (OpCode, RegDst, Jump, Branch, MemRead, MemtoReg, ALUOp, MemWrite, ALUSrc, RegWrite);
	input [5:0] OpCode;
	output [1:0] ALUOp;
	output RegDst, Jump, Branch, MemRead, MemtoReg, MemWrite, ALUSrc, RegWrite;

	// 000000 : add, sub, and, or, slt
	// 001000 : addi 
	// 100011 : lw
	// 101011 : sw
	// 000100 : beq
	// 000010 : j

	// 000000 (R-format)
	assign RegDst=(~OpCode[5])&(~OpCode[4])&(~OpCode[3])&(~OpCode[2])&(~OpCode[1])&(~OpCode[0]);
	// 000000 (R-format)
	assign ALUOp[1]=(~OpCode[5])&(~OpCode[4])&(~OpCode[3])&(~OpCode[2])&(~OpCode[1])&(~OpCode[0]);
	// 000100 (beq)
	assign ALUOp[0]=(~OpCode[5])&(~OpCode[4])&(~OpCode[3])&(OpCode[2])&(~OpCode[1])&(~OpCode[0]);
	// 100011 (lw), 101011 (sw)
	assign ALUSrc=((OpCode[5])&(~OpCode[4])&(~OpCode[3])&(~OpCode[2])&(OpCode[1])&(OpCode[0]))  | 
					  ((OpCode[5])&(~OpCode[4])&(OpCode[3])&(~OpCode[2])&(OpCode[1])&(OpCode[0])); 
	// 000100 (beq)
	assign Branch=(~OpCode[5])&(~OpCode[4])&(~OpCode[3])&(OpCode[2])&(~OpCode[1])&(~OpCode[0]);
	// 100011 (lw)
	assign MemRead=(OpCode[5])&(~OpCode[4])&(~OpCode[3])&(~OpCode[2])&(OpCode[1])&(OpCode[0]);
	// 101011 (sw)
	assign MemWrite=(OpCode[5])&(~OpCode[4])&(OpCode[3])&(~OpCode[2])&(OpCode[1])&(OpCode[0]);
	// 100011 (lw)
	assign MemtoReg=(OpCode[5])&(~OpCode[4])&(~OpCode[3])&(~OpCode[2])&(OpCode[1])&(OpCode[0]);
	// 000000 (R-format), 001000 (addi), 001100, 100011 (lw)
	assign RegWrite=((~OpCode[5])&(~OpCode[4])&(~OpCode[3])&(~OpCode[2])&(~OpCode[1])&(~OpCode[0]))|
 	                ((~OpCode[5])&(~OpCode[4])&(OpCode[3])&(~OpCode[2])&(~OpCode[1])&(~OpCode[0])) |
						 ((OpCode[5])&(~OpCode[4])&(~OpCode[3])&(~OpCode[2])&(OpCode[1])&(OpCode[0]));
	// 000010 (j)
	assign Jump=(~OpCode[5])&(~OpCode[4])&(~OpCode[3])&(~OpCode[2])&(OpCode[1])&(~OpCode[0]);
endmodule

module ALU_Control (ALUOp, f_code, operation_code);
	input [1:0] ALUOp;
	input [5:0] f_code;
	output [3:0] operation_code;
	
	assign operation_code[3]=0;
	// 0  1   | x x x x x x branch => subtract
	// 1  0   | x x 0 0 1 0 => R-type subtract
	// 1  0   | x x 1 0 1 0 => R-type slt
	assign operation_code[2]=((~ALUOp[1])&(ALUOp[0])) |
						((ALUOp[1])&(~ALUOp[0])&(~f_code[3])&(~f_code[2])&(f_code[1])&(~f_code[0])) |
						((ALUOp[1])&(~ALUOp[0])&(f_code[3])&(~f_code[2])&(f_code[1])&(~f_code[0]));
	// 0  0   | x x x x x x lw or sw => add	
	// 0  1   | x x x x x x branch => subtract
	// 1  0   | x x 0 0 0 0 => R-type add 
	// 1  0   | x x 0 0 1 0 => R-type subtract
	// 1  0   | x x 1 0 1 0 => R-type slt
	assign operation_code[1]=((~ALUOp[1])&(~ALUOp[0])) |
							   ((~ALUOp[1])&(ALUOp[0]))  |
								((ALUOp[1])&(~ALUOp[0])&(~f_code[3])&(~f_code[2])&(~f_code[1])&(~f_code[0])) |
								((ALUOp[1])&(~ALUOp[0])&(~f_code[3])&(~f_code[2])&(f_code[1])&(~f_code[0]))  | 
								((ALUOp[1])&(~ALUOp[0])&(f_code[3])&(~f_code[2])&(f_code[1])&(~f_code[0]));
	// ALU OP | f_code field
	// 1  0   | x x 0 1 0 1 => R-type Or 
	// 1  0   | x x 1 0 1 0 => R-type slt
	assign operation_code[0]=((ALUOp[1])&(~ALUOp[0])&(~f_code[3])&(f_code[2])&(~f_code[1])&(f_code[0])) | 
								((ALUOp[1])&(~ALUOp[0])&(f_code[3])&(~f_code[2])&(f_code[1])&(~f_code[0]));	

endmodule

// 32-bit ALU
module ALU (input1, input2, alu_out, zero, control);
	// TODO : negative number handling
	input [31:0] input1, input2;
	input [3:0] control;

	output reg [31:0] alu_out;
	output reg zero;

	always @ (control or input1 or input2) begin
		case (control)
			// and
			4'b0000: begin alu_out<=input1&input2; zero<=0; end
			// or
			4'b0001: begin alu_out<=input1|input2; zero<=0; end
			// add
			4'b0010: begin alu_out<=input1+input2; zero<=0; end
			// subtract
			4'b0110: begin 
				if(input1 == input2) 
					zero <= 1; 
				else 
					zero <= 0; 
					alu_out <= input1 - input2; 
				end
			// slt 
			4'b0111: begin 
				zero <= 0; 
				if(input1 - input2 >= 32'h8000_0000) 
					alu_out <= 32'b1; 
				else
					alu_out <= 32'b0; 
				end
		default: begin 
			zero <= 0; 
			alu_out <= input1;
		end
		endcase
	end
endmodule

// Referred Previous Memory Project 
module Data_Memory (MemAddr, Write_Data, Read_Data, clk, reset, MemRead, MemWrite);
	input clk, reset;
	input MemRead, MemWrite;

	input [7:0] MemAddr;
	input [31:0] Write_Data;

	output reg [31:0] Read_Data;

	reg [31:0] mem [63:0];
	integer k;

	always @(*) begin
		if (!reset) begin
			for (k = 0; k < 64; k = k + 1) begin
				mem[k] = 32'b0;
			end
			// mem[0xC] = 30;
			mem[3] = 30;
			mem[12] = 32'b0001_1110;
			mem[16] = 30;
			
		end

		else
			if (MemRead && !MemWrite) begin
				Read_Data = mem[MemAddr[7:2]];
			end

			else if (!MemRead && MemWrite) begin
				mem[MemAddr[7:2]] = Write_Data;
			end

		else begin
			Read_Data = 32'bx;
		end
	end
endmodule

module Forwarding_Unit (EX_RegisterRs, EX_RegisterRt, 
	MEM_RegisterRd, WB_RegisterRd, 
	EX_MEM_RegWrite, MEM_WB_RegWrite,
	// IF_ID_RegisterRs, IF_ID_RegisterRt, 
	ForwardA, ForwardB);
	// ForwardC, ForwardD);
	input [4:0] MEM_RegisterRd, WB_RegisterRd, EX_RegisterRs, EX_RegisterRt;
	// input [4:0] IF_ID_RegisterRs,IF_ID_RegisterRt;
	input EX_MEM_RegWrite, MEM_WB_RegWrite;
	// output ForwardC,ForwardD;
	output reg [1:0] ForwardA, ForwardB;
	// reg ForwardC,ForwardD;

	// wire equal_EXMEM_rs, equal_EXMEM_rt, equal_MEMWB_rs, equal_MEMWB_rt;
	// wire nonzero_EXMEM_rd,nonzero_MEMWB_rd;

	// assign nonzero_EXMEM_rd=(MEM_RegisterRd==0)?0:1;
	// assign nonzero_MEMWB_rd=(WB_RegisterRd==0)?0:1;
	// assign equal_EXMEM_rs=(MEM_RegisterRd==EX_RegisterRs)?1:0;
	// assign equal_EXMEM_rt=(MEM_RegisterRd==EX_RegisterRt)?1:0;
	// assign equal_MEMWB_rs=(WB_RegisterRd==EX_RegisterRs)?1:0;
	// assign equal_MEMWB_rt=(WB_RegisterRd==EX_RegisterRt)?1:0;
	// assign equal_WB_ID_rs=(WB_RegisterRd==IF_ID_RegisterRs)?1:0;
	// assign equal_WB_ID_rt=(WB_RegisterRd==IF_ID_RegisterRt)?1:0;

	// wire EX_HazardA, EX_HazardB;
	// wire WB_HazardA, WB_HazardB;
	
	assign EX_HazardA = EX_MEM_RegWrite & (MEM_RegisterRd!=0) & (MEM_RegisterRd==EX_RegisterRs);
	assign EX_HazardB = EX_MEM_RegWrite & (MEM_RegisterRd!=0) & (MEM_RegisterRd==EX_RegisterRt);
	assign WB_HazardA = MEM_WB_RegWrite & (WB_RegisterRd!=0) & (WB_RegisterRd==EX_RegisterRs);
	assign WB_HazardB = MEM_WB_RegWrite & (WB_RegisterRd!=0) & (WB_RegisterRd==EX_RegisterRt);


	always@ (EX_MEM_RegWrite or MEM_WB_RegWrite or EX_HazardA or WB_HazardA or EX_HazardB or WB_HazardB) begin
		
		if(EX_HazardA)
			ForwardA<=2'b10;
		else if(!EX_HazardA & WB_HazardA)
			ForwardA<=2'b01;
		else
			ForwardA<=2'b00;

		if(EX_HazardB)
			ForwardB<=2'b10;
		else if(!EX_HazardB & WB_HazardB)
			ForwardB<=2'b01;
		else
			ForwardB<=2'b00;
	end

	
	// always@ (EX_MEM_RegWrite or MEM_WB_RegWrite or 
	// nonzero_EXMEM_rd or nonzero_MEMWB_rd or equal_EXMEM_rs
	// or equal_EXMEM_rt or equal_MEMWB_rs or equal_MEMWB_rt or 
	// equal_WB_ID_rs or equal_WB_ID_rt) begin
	// 	if(EX_MEM_RegWrite & nonzero_EXMEM_rd & equal_EXMEM_rs)
	// 		ForwardA<=2'b10;
	// 	else if (MEM_WB_RegWrite & nonzero_MEMWB_rd & equal_MEMWB_rs)
	// 		ForwardA<=2'b01;
	// 	else 
	// 		ForwardA<=2'b00;
			
	// 	if(EX_MEM_RegWrite & nonzero_EXMEM_rd & equal_EXMEM_rt)
	// 		ForwardB<=2'b10;
	// 	else if (MEM_WB_RegWrite & nonzero_MEMWB_rd & equal_MEMWB_rt)
	// 		ForwardB<=2'b01;
	// 	else 
	// 		ForwardB<=2'b00;
			
	// 	if(MEM_WB_RegWrite & nonzero_MEMWB_rd & equal_WB_ID_rs)
	// 		ForwardC<=1;
	// 	else
	// 		ForwardC<=0;
			
    //   if(MEM_WB_RegWrite & nonzero_MEMWB_rd & equal_WB_ID_rt)
	// 		ForwardD<=1;
	// 	else
	// 		ForwardD<=0;
	// end

endmodule

module stall_for_lw_Control (ID_EX_RegisterRt, IF_ID_RegisterRs, IF_ID_RegisterRt, ID_EX_MemRead, PCWrite, IF_ID_Write, ID_Flush_lwstall);
	input [4:0] ID_EX_RegisterRt, IF_ID_RegisterRs, IF_ID_RegisterRt;
	input ID_EX_MemRead;
	output PCWrite, IF_ID_Write, ID_Flush_lwstall;
	wire equal_IDEXrt_IFIDrs,equal_IDEXrt_IFIDrt;
	assign equal_IDEXrt_IFIDrs=(ID_EX_RegisterRt==IF_ID_RegisterRs)?1:0;
	assign equal_IDEXrt_IFIDrt=(ID_EX_RegisterRt==IF_ID_RegisterRt)?1:0;
	reg PCWrite, IF_ID_Write, ID_Flush_lwstall;
	
	always@(ID_EX_MemRead or equal_IDEXrt_IFIDrs or equal_IDEXrt_IFIDrt)
	begin
		if(ID_EX_MemRead & (equal_IDEXrt_IFIDrs|equal_IDEXrt_IFIDrt))
		begin PCWrite<=0;IF_ID_Write<=0;ID_Flush_lwstall<=1; end
		else 
		begin PCWrite<=1;IF_ID_Write<=1;ID_Flush_lwstall<=0; end
	end
endmodule

module Mux_32bit_3to1 (in00, in01, in10, mux_out, control);
	input [31:0] in00, in01, in10;
	output [31:0] mux_out;
	input [1:0] control;
	reg [31:0] mux_out;
	always @(in00 or in01 or in10 or control)
	begin
		case(control)
		2'b00:mux_out<=in00;
		2'b01:mux_out<=in01;
		2'b10:mux_out<=in10;
		default: mux_out<=in00;
		endcase
	end 
endmodule