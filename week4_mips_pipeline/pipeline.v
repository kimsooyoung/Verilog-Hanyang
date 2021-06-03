`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date:    13:14:17 06/02/2021 
// Design Name: 
// Module Name:    single_cycle_main 
// Project Name: 
// Target Devices: 
// Tool versions: 
// Description: 
//
// Dependencies: 
//
// Revision: 
// Revision 0.01 - File Created
// Additional Comments: https://github.com/hxing9974/Verilog-Pipeline-Processor/blob/master/pipeline_v10_demo.v#L130
// 
//////////////////////////////////////////////////////////////////////////////////


module single_cycle (clk, reset, result);

	wire [6:0] PC_out_short; // PC

	wire IF_ID_Write;				// IF/ID reg stops writing if IF_ID_Write == 0


	Program_Counter Unit0 (.clk(clk), .reset(reset), .PC_in(PC_in_original[6:0]), .PC_out(PC_out_short), .PCWrite(PCWrite));
	Instruction_Memory Unit1 (.read_addr(PC_out_short), .instruction(instruction), .reset(reset));

	// fetch
	assign PC_out_unsign_extended = {26'b0000_0000_0000_0000_0000_0000_0, PC_out_short}; // from 7 bits to 32 bits PC_out_short = PC 6bit
	ALU_add_only Unit2 (.inA(PC_out_unsign_extended), .inB(32'b0100), .add_out(PC_plus4)); // PC + 4
	Mux_N_bit #(32) Unit3 (.in0(PC_plus4), .in1(branch_jump_addr), .mux_out(PC_in_original), .control(PCSrc)); // PC + 4 ? or Branch ?
	IF_ID_Stage_Reg Unit4 (.PC_plus4_in(PC_plus4), .PC_plus4_out(IF_ID_PC_plus4), .instruction_in(instruction), 
		.instruction_out(IF_ID_instruction), .IF_ID_Write(IF_ID_Write), .IF_Flush(IF_Flush), .clk(clk), .reset(reset));

	// decode
	// 6 5 5 
	// multi_purpose_read_addr 5 bit, 
	Register_File Unit5 (.read_addr_1(multi_purpose_read_addr), .read_addr_2(IF_ID_instruction[20:16]), 
		.write_addr(MEM_WB_RegisterRd), .read_data_1(reg_read_data_1), .read_data_2(reg_read_data_2), .write_data(reg_write_data), 
		.RegWrite(multi_purpose_RegWrite), .clk(clkRF), .reset(reset));
	Sign_Extension Unit6 (.sign_in(IF_ID_instruction[15:0]), .sign_out(immi_sign_extended)); // jump within ID stage
	// Shift in here? => Yes for Jump
	Shift_Left_2_Jump Unit7 (.shift_in(IF_ID_instruction[25:0]), .shift_out(jump_base28));
	assign jump_addr = {IF_ID_PC_plus4[31:28], jump_base28}; // jump_addr = (PC+4)[31:28] joined with jump_base28[27:0]
	Control Unit8 (.OpCode(IF_ID_instruction[31:26]), .RegDst(RegDst), .Jump(Jump), .Branch(Branch), 
		.MemRead(MemRead), .MemtoReg(MemtoReg), .ALUOp(ALUOp),
		.MemWrite(MemWrite), .ALUSrc(ALUSrc), .RegWrite(RegWrite));
	ID_EX_Stage_Reg Unit9 (.ID_Flush_lwstall(ID_Flush_lwstall), .ID_Flush_Branch(ID_Flush_Branch), .RegWrite_in(RegWrite), 
		.MemtoReg_in(MemtoReg), .RegWrite_out(ID_EX_RegWrite), .MemtoReg_out(ID_EX_MemtoReg), .Branch_in(Branch), 
		.MemRead_in(MemRead), .MemWrite_in(MemWrite), .Jump_in(Jump), .Branch_out(ID_EX_Branch), .MemRead_out(ID_EX_MemRead), 
		.MemWrite_out(ID_EX_MemWrite), .Jump_out(ID_EX_Jump), .RegDst_in(RegDst), .ALUSrc_in(ALUSrc), .RegDst_out(ID_EX_RegDst),
		.ALUSrc_out(ID_EX_ALUSrc), .ALUOp_in(ALUOp), .ALUOp_out(ID_EX_ALUOp), .jump_addr_in(jump_addr), .PC_plus4_in(IF_ID_PC_plus4), 
		.jump_addr_out(ID_EX_jump_addr), .PC_plus4_out(ID_EX_PC_plus4), .reg_read_data_1_in(muxC_out), .reg_read_data_2_in(muxD_out), 
		.immi_sign_extended_in(immi_sign_extended), .reg_read_data_1_out(ID_EX_reg_read_data_1), .reg_read_data_2_out(ID_EX_reg_read_data_2), 
		.immi_sign_extended_out(ID_EX_immi_sign_extended), .IF_ID_RegisterRs_in(IF_ID_instruction[25:21]), .IF_ID_RegisterRt_in(IF_ID_instruction[20:16]),
		.IF_ID_RegisterRd_in(IF_ID_instruction[15:11]), .IF_ID_RegisterRs_out(ID_EX_RegisterRs), .IF_ID_RegisterRt_out(ID_EX_RegisterRt), 
		.IF_ID_RegisterRd_out(ID_EX_RegisterRd), .IF_ID_funct_in(IF_ID_instruction[5:0]),.IF_ID_funct_out(ID_EX_funct),.clk(clk), .reset(reset));//Ou adds  IF_ID_funct_in and out

	// EX stage
	Mux_N_bit #(5) Unit10 (.in0(ID_EX_RegisterRt), .in1(ID_EX_RegisterRd), .mux_out(EX_RegisterRd), .control(ID_EX_RegDst));
	ALUControl Unit11 (.ALUOp(ID_EX_ALUOp), .funct(ID_EX_funct), .out_to_ALU(out_to_ALU));
	// Ou modifies: funct should not be IF_ID_instruction. Rather, it should be ID_EX_funct(a new wire)
	// forwarding => eject
	Mux_32bit_3to1 Unit12_muxA (.in00(ID_EX_reg_read_data_1), .in01(reg_write_data), .in10(EX_MEM_ALU_result), .mux_out(muxA_out), .control(ForwardA));
	Mux_32bit_3to1 Unit13_muxB (.in00(ID_EX_reg_read_data_2), .in01(reg_write_data), .in10(EX_MEM_ALU_result), .mux_out(muxB_out), .control(ForwardB));//Ou modifies: keep the structure paralleled with muxA
	Mux_N_bit #(32) Unit14 (.in0(muxB_out), .in1(ID_EX_immi_sign_extended), .mux_out(after_ALUSrc), .control(ID_EX_ALUSrc));
	ALU Unit15 (.inA(muxA_out), .inB(after_ALUSrc), .alu_out(ALU_result), .zero(ALU_zero), .control(out_to_ALU));
	Shift_Left_2_Branch Unit16 (.shift_in(ID_EX_immi_sign_extended), .shift_out(after_shift));
	ALU_add_only Unit17 (.inA(ID_EX_PC_plus4), .inB(after_shift), .add_out(branch_addr)); // (PC+4) + branch_addition*4Z

	// in EX/MEM stage reg, note muxB_out is used in the place of reg_read_data_2 as a result of forwarding;ygb 
	EX_MEM_Stage_Reg Unit18 (.EX_Flush(EX_Flush), .RegWrite_in(ID_EX_RegWrite), .MemtoReg_in(ID_EX_MemtoReg), 
	.RegWrite_out(EX_MEM_RegWrite), .MemtoReg_out(EX_MEM_MemtoReg),
	.Branch_in(ID_EX_Branch), .MemRead_in(ID_EX_MemRead), .MemWrite_in(ID_EX_MemWrite),
	.Jump_in(ID_EX_Jump), .Branch_out(EX_MEM_Branch), .MemRead_out(EX_MEM_MemRead),
	.MemWrite_out(EX_MEM_MemWrite), .Jump_out(EX_MEM_Jump), .jump_addr_in(ID_EX_jump_addr), 
	.branch_addr_in(branch_addr), .jump_addr_out(EX_MEM_jump_addr), 
	.branch_addr_out(EX_MEM_branch_addr), .ALU_zero_in(ALU_zero),
	.ALU_zero_out(EX_MEM_ALU_zero), .ALU_result_in(ALU_result), .reg_read_data_2_in(muxB_out), 
	.ALU_result_out(EX_MEM_ALU_result), .reg_read_data_2_out(EX_MEM_reg_read_data_2), 
	.ID_EX_RegisterRd_in(EX_RegisterRd), .EX_MEM_RegisterRd_out(EX_MEM_RegisterRd), .clk(clk), .reset(reset));

	// MEM stage
	Data_Memory Unit19 (.addr(EX_MEM_ALU_result[7:0]), .write_data(EX_MEM_reg_read_data_2), .read_data(D_MEM_data), 
		.clk(clk), .reset(reset), .MemRead(EX_MEM_MemRead), .MemWrite(EX_MEM_MemWrite));
	and (Branch_taken, EX_MEM_Branch, EX_MEM_ALU_zero);
	jump_OR_branch Unit20 (.Jump(EX_MEM_Jump), .Branch_taken(Branch_taken), .branch_addr(EX_MEM_branch_addr), 
		.jump_addr(EX_MEM_jump_addr), .PCSrc(PCSrc), .addr_out(branch_jump_addr));

	

	always @(clk) begin
		if (switchRun) begin
			// sys status 1: run pipeline processor
			clkRF_reg <= clkNormal;		// 1 Hz
			clk_reg <= clkNormal;		// 1 Hz
			multi_purpose_read_addr_reg <= IF_ID_instruction[25:21]; // reg-file-port1 reads from instruction
			// reg-file protection measure; explained in "else"
			multi_purpose_RegWrite_reg <= MEM_WB_RegWrite;
			// output PC to SSD, but since PC only has 6 bits
			tho_reg <= PC_out_unsign_extended[15:12];	// always 0
			hun_reg <= PC_out_unsign_extended[11:8];	// always 0
			ten_reg <= PC_out_unsign_extended[7:4];
			one_reg <= PC_out_unsign_extended[3:0];
		end
		else begin
			// sys status 2: pause processor; inspect Reg File content
			clkRF_reg <= clkSSD;	// 500 Hz
			clk_reg <= 1'b0;		// freeze at 0
			multi_purpose_read_addr_reg <= SwitchSelector; // reg-file-port1 reads from SwitchSelector
			// Reg-file is not freezed in time, this protects against RF-data-overwrite
			multi_purpose_RegWrite_reg <= 1'b0;
			// output reg file content to SSD, but only the lower 16 bits (we only have 4 SSD)
			tho_reg <= reg_read_data_1[15:12];
			hun_reg <= reg_read_data_1[11:8];
			ten_reg <= reg_read_data_1[7:4];
			one_reg <= reg_read_data_1[3:0];
		end
	end
endmodule

//////////////////////////////////////////////////////////////////////////
// modules that require modification from single-cycle implementation ////
//////////////////////////////////////////////////////////////////////////
// this one added a PCWrite signal. 
// Stop writing on this rising edge if PCWrite equals 0
//
// original module description:
// rising-edge synchronous program counter
// output range: decimal 0 to 32 (== I-MEM height)
// data I/O width: 64 = 2^6
// async reset: set program counter to 0 asynchronously
module Program_Counter (clk, reset, PC_in, PC_out, PCWrite);
	input PCWrite; // new input for pipeline
	input clk, reset;
	input [6:0] PC_in;//at most 32 instructions
	output [6:0] PC_out;
	reg [6:0] PC_out;
	always @ (posedge clk or posedge reset)
	begin
		if(reset==1'b1)
			PC_out<=0;
		else if(PCWrite)
			PC_out<=PC_in;
	end
endmodule

//////////////////////////////////////////////////////////////////////////
// modules that are direct copies from single cycle implementation    ////
//////////////////////////////////////////////////////////////////////////
// async read I-MEM
// height: 64, width: 32 bits (as required by TA)
// PC input width: 64 = 2^6
// instruction output width: 32 bits (== I-MEM width)
// async reset: as specified in document "Project Two Specification (V3)", 
// 		  		first reset all to 0, then hard-code instructions
module Instruction_Memory (read_addr, instruction, reset);
	input reset;
	input [6:0] read_addr;
	output [31:0] instruction;
	reg [31:0] Imemory [63:0];
	integer k;
	// I-MEM in this case is addressed by word, not by byte
	wire [4:0] shifted_read_addr;
	assign shifted_read_addr = read_addr[6:2];
	assign instruction = Imemory[shifted_read_addr];

	always @(posedge reset)
	begin
		for (k=0; k<30; k=k+1) begin// here Ou changes k=0 to k=16
			Imemory[k] = 32'b0;
		end
					
		Imemory[0] = 32'b001000_00000_01000_0000000000100000; //addi $t0, $zero, 32 
		Imemory[1] = 32'b001000_00000_010010_000000000110111; //addi $t1, $zero, 55 
		Imemory[2] = 32'b00000001000010011000000000100100; //and $s0, $t0, $t1 
		Imemory[3] = 32'b00000001000010011000000000100101; //or $s0, $t0, $t1 
		Imemory[4] = 32'b10101100000100000000000000000100; //sw $s0, 4($zero) 
		Imemory[5] = 32'b10101100000010000000000000001000; //sw $t0, 8($zero) 
		Imemory[6] = 32'b00000001000010011000100000100000; //add $s1, $t0, $t1 
		Imemory[7] = 32'b00000001000010011001000000100010; //sub $s2, $t0, $t1 
		Imemory[8] = 32'b00010010001100100000000000001001; //beq $s1, $s2, error0 
		Imemory[9] = 32'b10001100000100010000000000000100; //lw $s1, 4($zero) 
		Imemory[10]= 32'b00110010001100100000000001001000; //andi $s2, $s1, 48 
		Imemory[11] =32'b00010010001100100000000000001001; //beq $s1, $s2, error1 
		Imemory[12] =32'b10001100000100110000000000001000; //lw $s3, 8($zero) 
		Imemory[13] =32'b00010010000100110000000000001010; //beq $s0, $s3, error2 
		Imemory[14] =32'b00000010010100011010000000101010; //slt $s4, $s2, $s1 (Last) 
		Imemory[15] =32'b00010010100000000000000000001111; //beq $s4, $0, EXIT 
		Imemory[16] =32'b00000010001000001001000000100000; //add $s2, $s1, $0 
		Imemory[17] =32'b00001000000000000000000000001110; //j Last
		Imemory[18] =32'b00100000000010000000000000000000; //addi $t0, $0, 0(error0) 
		Imemory[19] =32'b00100000000010010000000000000000; //addi $t1, $0, 0 
		Imemory[20] =32'b00001000000000000000000000011111; //j EXIT
		Imemory[21] =32'b00100000000010000000000000000001; //addi $t0, $0, 1(error1) 
		Imemory[22] =32'b00100000000010010000000000000001; //addi $t1, $0, 1 
		Imemory[23] =32'b00001000000000000000000000011111; //j EXIT
		Imemory[24] =32'b00100000000010000000000000000010; //addi $t0, $0, 2(error2) 
		Imemory[25] =32'b00100000000010010000000000000010; //addi $t1, $0, 2 
		Imemory[26] =32'b00001000000000000000000000011111; //j EXIT
		Imemory[27] =32'b00100000000010000000000000000011; //addi $t0, $0, 3(error3) 
		Imemory[28] =32'b00100000000010010000000000000011; //addi $t1, $0, 3 
		Imemory[29] =32'b00001000000000000000000000011111; //j EXIT
			
	end
endmodule

// 32-bit ALU for addition only
// data input width: 2 32-bit
// data output width: 1 32-bit, no "zero" output
// control: no control input, only addition operation implemeneted
// as specified in Fig 4.12
// attached in email as "Fig 4_12 ALU Control Input"
module ALU_add_only (inA, inB, add_out);
	input [31:0] inA, inB;
	output [31:0] add_out;
	assign add_out=inA+inB;
endmodule

// IF/ID stage register
// update content & output updated content at rising edge
module IF_ID_Stage_Reg (PC_plus4_in, PC_plus4_out, instruction_in, instruction_out, IF_ID_Write, IF_Flush, clk, reset);
	// 1. data content
	input [31:0] PC_plus4_in, instruction_in;
	output [31:0] PC_plus4_out, instruction_out;
	// 2. hazard control
	// IF_ID_Write: sync; if (IF_ID_Write==1'b0), do not update content at this rising edge
	// IF_Flush: sync; if (IF_Flush==1), clear ALL content, NOT ONLY control signals
	input IF_ID_Write, IF_Flush;
	// 3. general contorl
	// reset: async; set all register content to 0
	input clk, reset;

	reg [31:0] PC_plus4_out, instruction_out;

	always @(posedge clk or posedge reset)
	begin
		if (reset==1'b1)
		begin
			PC_plus4_out <= 32'b0;
			instruction_out <= 32'b0;
		end
		else if (IF_Flush==1'b1)
		begin
			PC_plus4_out <= 32'b0;
			instruction_out <= 32'b0;
		end
		else if (IF_ID_Write==1'b1)
		begin
			PC_plus4_out <= PC_plus4_in;
			instruction_out <= instruction_in;
		end

	end
	
endmodule

// sync register file (write/read occupy half cycle each)
// height: 32 (from $0 to $ra), width: 32 bits
// write: on rising edge; data width 32 bit; address width 5 bit
// read: on falling edge; data width 32 bit; address width 5 bit
// control: write on rising edge if (RegWrite == 1)
// async reset: set all register content to 0

// Register_File Unit5 (.read_addr_1(multi_purpose_read_addr), .read_addr_2(IF_ID_instruction[20:16]), 
// 	.write_addr(MEM_WB_RegisterRd), .read_data_1(reg_read_data_1), .read_data_2(reg_read_data_2), .write_data(reg_write_data), 
// 	.RegWrite(multi_purpose_RegWrite), .clk(clkRF), .reset(reset));
module Register_File (read_addr_1, read_addr_2, write_addr, read_data_1, read_data_2, write_data, RegWrite, clk, reset);
	input [4:0] read_addr_1, read_addr_2, write_addr;
	input [31:0] write_data;
	input clk, reset, RegWrite;
	output [31:0] read_data_1, read_data_2;

	reg [31:0] Regfile [31:0];
	integer k;
	
	assign read_data_1 = Regfile[read_addr_1];
	assign read_data_2 = Regfile[read_addr_2];

	always @(posedge clk or posedge reset) // Ou combines the block of reset into the block of posedge clk
	begin
		if (reset==1'b1)
		begin
			for (k=0; k<32; k=k+1) 
			begin
				Regfile[k] = 32'b0;
			end
		end 
		
		else if (RegWrite == 1'b1) Regfile[write_addr] = write_data; 
	end

endmodule

// sign-extend the 16-bit input to the 32_bit output
module Sign_Extension (sign_in, sign_out);
	input [15:0] sign_in;
	output [31:0] sign_out;
	assign sign_out[15:0]=sign_in[15:0];
	assign sign_out[31:16]=sign_in[15]?16'b1111_1111_1111_1111:16'b0;
endmodule

// shift-left-2 for jump instruction
// input width: 26 bits
// output width: 28 bits
// fill the void with 0 after shifting
// we don't need to shift in this case, becasue the address of the instructions
// are addressed by words
module Shift_Left_2_Jump (shift_in, shift_out);
	input [25:0] shift_in;
	output [27:0] shift_out;
	assign shift_out[27:0]={shift_in[25:0],2'b00};
endmodule

// async control signal generation unit based on OpCode
// as specified in Fig 4.22
// attached in email as file "Fig 4_22 Single Cycle Control"
// input: 6 bits OpCode
// output: all 1 bit except ALUOp which is 2-bits wide
module Control (OpCode, RegDst, Jump, Branch, MemRead, MemtoReg, ALUOp, MemWrite, ALUSrc, RegWrite);
	input [5:0] OpCode;
	output RegDst, Jump, Branch, MemRead, MemtoReg, MemWrite, ALUSrc, RegWrite;
	output [1:0] ALUOp;

	assign RegDst=(~OpCode[5])&(~OpCode[4])&(~OpCode[3])&(~OpCode[2])&(~OpCode[1])&(~OpCode[0]);//000000
	assign Jump=(~OpCode[5])&(~OpCode[4])&(~OpCode[3])&(~OpCode[2])&(OpCode[1])&(~OpCode[0]);//000010
	assign Branch=(~OpCode[5])&(~OpCode[4])&(~OpCode[3])&(OpCode[2])&(~OpCode[1])&(~OpCode[0]);//000100
	assign MemRead=(OpCode[5])&(~OpCode[4])&(~OpCode[3])&(~OpCode[2])&(OpCode[1])&(OpCode[0]);//100011
	assign MemtoReg=(OpCode[5])&(~OpCode[4])&(~OpCode[3])&(~OpCode[2])&(OpCode[1])&(OpCode[0]);//100011
	assign MemWrite=(OpCode[5])&(~OpCode[4])&(OpCode[3])&(~OpCode[2])&(OpCode[1])&(OpCode[0]);//101011
	
	// what other operator?
	assign ALUSrc=((~OpCode[5])&(~OpCode[4])&(OpCode[3])&(~OpCode[2])&(~OpCode[1])&(~OpCode[0])) | ((~OpCode[5])&(~OpCode[4])&(OpCode[3])&(OpCode[2])&(~OpCode[1])&(~OpCode[0])) | ((OpCode[5])&(~OpCode[4])&(~OpCode[3])&(~OpCode[2])&(OpCode[1])&(OpCode[0])) | (((OpCode[5])&(~OpCode[4])&(OpCode[3])&(~OpCode[2])&(OpCode[1])&(OpCode[0]))); //001000,001100,100011,101011
	assign RegWrite=(~OpCode[5])&(~OpCode[4])&(~OpCode[3])&(~OpCode[2])&(~OpCode[1])&(~OpCode[0]) | ((~OpCode[5])&(~OpCode[4])&(OpCode[3])&(~OpCode[2])&(~OpCode[1])&(~OpCode[0])) | ((~OpCode[5])&(~OpCode[4])&(OpCode[3])&(OpCode[2])&(~OpCode[1])&(~OpCode[0])) | ((OpCode[5])&(~OpCode[4])&(~OpCode[3])&(~OpCode[2])&(OpCode[1])&(OpCode[0]));//000000,001000,001100,100011
	assign ALUOp[1]=((~OpCode[5])&(~OpCode[4])&(~OpCode[3])&(~OpCode[2])&(~OpCode[1])&(~OpCode[0]))|((~OpCode[5])&(~OpCode[4])&(OpCode[3])&(OpCode[2])&(~OpCode[1])&(~OpCode[0]));//000000, 001100(andi)
	assign ALUOp[0]= ((~OpCode[5])&(~OpCode[4])&(~OpCode[3])&(OpCode[2])&(~OpCode[1])&(~OpCode[0]))|((~OpCode[5])&(~OpCode[4])&(OpCode[3])&(OpCode[2])&(~OpCode[1])&(~OpCode[0]));//000100,001100(andi)
endmodule

// ID/EX stage register
// update content & output updated content at rising edge
module ID_EX_Stage_Reg (ID_Flush_lwstall, ID_Flush_Branch, RegWrite_in, MemtoReg_in, RegWrite_out, MemtoReg_out, Branch_in, MemRead_in,
	MemWrite_in, Jump_in, Branch_out, MemRead_out, MemWrite_out, Jump_out, RegDst_in, ALUSrc_in, RegDst_out, ALUSrc_out, ALUOp_in, ALUOp_out,
	jump_addr_in, PC_plus4_in, jump_addr_out, PC_plus4_out, reg_read_data_1_in, reg_read_data_2_in, immi_sign_extended_in, reg_read_data_1_out, 
	reg_read_data_2_out, immi_sign_extended_out, IF_ID_RegisterRs_in, IF_ID_RegisterRt_in, IF_ID_RegisterRd_in, IF_ID_RegisterRs_out, 
	IF_ID_RegisterRt_out, IF_ID_RegisterRd_out,IF_ID_funct_in, IF_ID_funct_out,clk, reset);
	// 1. hazard control signal (sync rising edge)
	// if either ID_Flush_lwstall or ID_Flush_Branch equals 1,
	// then clear all WB, MEM and EX control signal to 0 on rising edge
	// do not need to clear addr, data or reg content
	input ID_Flush_lwstall, ID_Flush_Branch;
	// 2. WB control signal
	input RegWrite_in, MemtoReg_in;
	output RegWrite_out, MemtoReg_out;
	// 3. MEM control signal
	input Branch_in, MemRead_in, MemWrite_in, Jump_in;
	output Branch_out, MemRead_out, MemWrite_out, Jump_out;
	// 4. EX control signal
	input RegDst_in, ALUSrc_in;
	input [1:0] ALUOp_in;
	output RegDst_out, ALUSrc_out;
	output [1:0] ALUOp_out;
	// 5. addr content
	input [31:0] jump_addr_in, PC_plus4_in;
	output [31:0] jump_addr_out, PC_plus4_out;
	// 6. data content
	input [31:0] reg_read_data_1_in, reg_read_data_2_in, immi_sign_extended_in;
	output [31:0] reg_read_data_1_out, reg_read_data_2_out, immi_sign_extended_out;
	// 7. reg content
	input [4:0] IF_ID_RegisterRs_in, IF_ID_RegisterRt_in, IF_ID_RegisterRd_in;
	output [4:0] IF_ID_RegisterRs_out, IF_ID_RegisterRt_out, IF_ID_RegisterRd_out;
	input [5:0] IF_ID_funct_in;
	output [5:0] IF_ID_funct_out;
	// general signal
	// reset: async; set all register content to 0
	input clk, reset;
	
	reg RegWrite_out, MemtoReg_out;
	reg Branch_out, MemRead_out, MemWrite_out, Jump_out;
	reg RegDst_out, ALUSrc_out;
	reg [1:0] ALUOp_out;
	reg [31:0] jump_addr_out, PC_plus4_out;
	reg [31:0] reg_read_data_1_out, reg_read_data_2_out, immi_sign_extended_out;
	reg [4:0] IF_ID_RegisterRs_out, IF_ID_RegisterRt_out, IF_ID_RegisterRd_out;
	reg [5:0] IF_ID_funct_out;
	
	always @(posedge clk or posedge reset)
	begin
		if (reset == 1'b1)
		begin
			RegWrite_out = 1'b0;
			MemtoReg_out = 1'b0;
			Branch_out = 1'b0;
			MemRead_out = 1'b0;
			MemWrite_out = 1'b0;
			Jump_out = 1'b0;
			RegDst_out = 1'b0;
			ALUSrc_out = 1'b0;
			ALUOp_out = 2'b0;
			jump_addr_out = 32'b0;
			PC_plus4_out = 32'b0;
			reg_read_data_1_out = 32'b0;
			reg_read_data_2_out = 32'b0;
			immi_sign_extended_out = 32'b0;
			IF_ID_RegisterRs_out = 5'b0;
			IF_ID_RegisterRt_out = 5'b0;
			IF_ID_RegisterRd_out = 5'b0;
			IF_ID_funct_out = 6'b0;			
		end
		else if (ID_Flush_lwstall == 1'b1)
		begin
			RegWrite_out = 1'b0;
			MemtoReg_out = 1'b0;
			Branch_out = 1'b0;
			MemRead_out = 1'b0;
			MemWrite_out = 1'b0;
			Jump_out = 1'b0;
			RegDst_out = 1'b0;
			ALUSrc_out = 1'b0;
			ALUOp_out = 2'b0;
		end
		else if (ID_Flush_Branch == 1'b1)
		begin
			RegWrite_out = 1'b0;
			MemtoReg_out = 1'b0;
			Branch_out = 1'b0;
			MemRead_out = 1'b0;
			MemWrite_out = 1'b0;
			Jump_out = 1'b0;
			RegDst_out = 1'b0;
			ALUSrc_out = 1'b0;
			ALUOp_out = 2'b0;
		end
		else begin
			RegWrite_out = RegWrite_in;
			MemtoReg_out = MemtoReg_in;
			Branch_out = Branch_in;
			MemRead_out = MemRead_in;
			MemWrite_out = MemWrite_in;
			Jump_out = Jump_in;
			RegDst_out = RegDst_in;
			ALUSrc_out = ALUSrc_in;
			ALUOp_out = ALUOp_in;
			jump_addr_out = jump_addr_in;
			PC_plus4_out = PC_plus4_in;
			reg_read_data_1_out = reg_read_data_1_in;
			reg_read_data_2_out = reg_read_data_2_in;
			immi_sign_extended_out = immi_sign_extended_in;
			IF_ID_RegisterRs_out = IF_ID_RegisterRs_in;
			IF_ID_RegisterRt_out = IF_ID_RegisterRt_in;
			IF_ID_RegisterRd_out = IF_ID_RegisterRd_in;
			IF_ID_funct_out = IF_ID_funct_in;
		end	
		
	end	
	
endmodule

module ALUControl (ALUOp, funct, out_to_ALU);
	input [1:0] ALUOp;
	input [5:0] funct;
	output [3:0] out_to_ALU;

	assign out_to_ALU[3]=0;
	assign out_to_ALU[2]=((~ALUOp[1])&(ALUOp[0])) | ((ALUOp[1])&(~ALUOp[0])&(~funct[3])&(~funct[2])&(funct[1])&(~funct[0])) | ((ALUOp[1])&(~ALUOp[0])&(funct[3])&(~funct[2])&(funct[1])&(~funct[0]));
	assign out_to_ALU[1]=((~ALUOp[1])&(~ALUOp[0]))|((~ALUOp[1])&(ALUOp[0])) | ((ALUOp[1])&(~ALUOp[0])&(~funct[3])&(~funct[2])&(~funct[1])&(~funct[0])) | ((ALUOp[1])&(~ALUOp[0])&(~funct[3])&(~funct[2])&(funct[1])&(~funct[0]))|((ALUOp[1])&(~ALUOp[0])&(funct[3])&(~funct[2])&(funct[1])&(~funct[0]));
	assign out_to_ALU[0]=((ALUOp[1])&(~ALUOp[0])&(~funct[3])&(funct[2])&(~funct[1])&(funct[0]))|((ALUOp[1])&(~ALUOp[0])&(funct[3])&(~funct[2])&(funct[1])&(~funct[0]));	
endmodule

// 32-bit ALU
// data input width: 2 32-bit
// data output width: 1 32-bit and one "zero" output
// control: 4-bit
// zero: output 1 if all bits of data output is 0
// as specified in Fig 4.12
// attached in email as "Fig 4_12 ALU Control Input"
module ALU (inA, inB, alu_out, zero, control);
	input [31:0] inA, inB;
	output [31:0] alu_out;
	output zero;
	reg zero;
	reg [31:0] alu_out;
	input [3:0] control;
	always @ (control or inA or inB)
	begin
		case (control)
		4'b0000:begin zero<=0; alu_out<=inA&inB; end
		4'b0001:begin zero<=0; alu_out<=inA|inB; end
		4'b0010:begin zero<=0; alu_out<=inA+inB; end
		4'b0110:begin if(inA==inB) zero<=1; else zero<=0; alu_out<=inA-inB; end
		4'b0111:begin zero<=0; if(inA-inB>=32'h8000_0000) alu_out<=32'b1; else alu_out<=32'b0; end// how to implement signed number
		default: begin zero<=0; alu_out<=inA; end
		endcase
	end
endmodule


// EX/MEM stage register
// update content & output updated content at rising edge
module EX_MEM_Stage_Reg (EX_Flush, RegWrite_in, MemtoReg_in, RegWrite_out, MemtoReg_out, Branch_in, MemRead_in, MemWrite_in, Jump_in, Branch_out, MemRead_out, MemWrite_out, Jump_out, jump_addr_in, branch_addr_in, jump_addr_out, branch_addr_out, ALU_zero_in, ALU_zero_out, ALU_result_in, reg_read_data_2_in, ALU_result_out, reg_read_data_2_out, ID_EX_RegisterRd_in, EX_MEM_RegisterRd_out, clk, reset);
	// 1. hazard control signal (sync rising edge)
	// if EX_Flush equals 1,
	// then clear all WB, MEM control signal to 0 on rising edge
	// do not need to clear addr or data content
	input EX_Flush;
	// 2. WB control signal
	input RegWrite_in, MemtoReg_in;
	output RegWrite_out, MemtoReg_out;
	// 3. MEM control signal
	input Branch_in, MemRead_in, MemWrite_in, Jump_in;
	output Branch_out, MemRead_out, MemWrite_out, Jump_out;
	// 4. addr content
	input [31:0] jump_addr_in, branch_addr_in;
	output [31:0] jump_addr_out, branch_addr_out;
	// 5. data content
	input ALU_zero_in;
	output ALU_zero_out;
	input [31:0] ALU_result_in, reg_read_data_2_in;
	output [31:0] ALU_result_out, reg_read_data_2_out;
	input [4:0] ID_EX_RegisterRd_in;
	output [4:0] EX_MEM_RegisterRd_out;
	// general signal
	// reset: async; set all register content to 0
	input clk, reset;

	reg RegWrite_out, MemtoReg_out;
	reg Branch_out, MemRead_out, MemWrite_out, Jump_out;
	reg [31:0] jump_addr_out, branch_addr_out;
	reg ALU_zero_out;
	reg [31:0] ALU_result_out, reg_read_data_2_out;
	reg [4:0] EX_MEM_RegisterRd_out;

	always @(posedge clk or posedge reset)
	begin
		if (reset == 1'b1)
		begin
		  RegWrite_out <= 1'b0;
		  MemtoReg_out <= 1'b0;
		  Branch_out <= 1'b0;
		  MemRead_out <= 1'b0;
		  MemWrite_out <= 1'b0;
		  Jump_out <= 1'b0;
		  jump_addr_out <= 32'b0;
		  branch_addr_out <= 32'b0;
		  ALU_zero_out <= 1'b0;
		  ALU_result_out <= 32'b0;
		  reg_read_data_2_out <= 32'b0;
		  EX_MEM_RegisterRd_out <= 5'b0; 
		end
		else if (EX_Flush == 1'b1)
	    begin
		  RegWrite_out <= 1'b0;
		  MemtoReg_out <= 1'b0;
		  Branch_out <= 1'b0;
		  MemRead_out <= 1'b0;
		  MemWrite_out <= 1'b0;
		  Jump_out <= 1'b0;
		end
		else begin
		  RegWrite_out <= RegWrite_in;
		  MemtoReg_out <= MemtoReg_in;
		  Branch_out <= Branch_in;
		  MemRead_out <= MemRead_in;
		  MemWrite_out <= MemWrite_in;
		  Jump_out <= Jump_in;
		  jump_addr_out <= jump_addr_in;
		  branch_addr_out <= branch_addr_in;
		  ALU_zero_out <= ALU_zero_in;
		  ALU_result_out <= ALU_result_in;
		  reg_read_data_2_out <= reg_read_data_2_in;
		  EX_MEM_RegisterRd_out <= ID_EX_RegisterRd_in;
		end

	end

endmodule
