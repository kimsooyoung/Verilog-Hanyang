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
	input	[4:0] key,
	input RXD,
	output TXD,
	output reg [15:0] led,
	output reg [3:0] digit,
	output reg [7:0] fnd,
	output reg [4:0] keyLed,
	output reg buzz
    );
	 
	reg [24:0] counter;
	reg [3:0] number=4'b0;
	reg [1:0] digit_sel=2'b0;
	reg [8:0] alert_num;
	reg [2:0] state;
	
	parameter 
	st_Init = 3'b100,
	st_BuzzOn = 3'b010, 
	st_BuzzOff = 2'b001;
	
	//uart register & parameter
	parameter CLOCKS_PER_BIT = 1302; // baud rate : 38400, Clock : 50MHz
	parameter CLOCKS_WAIT_FOR_RECEIVE = 651;
	parameter MAX_TX_BIT_COUNT = 9;
	parameter MAX_DATA_BUFFER_INDEX = 10;

	reg [10:0] tx_clk_count=0; // clock count
	reg [3:0] tx_bit_count=0; // bit count [start bit | d0 | d1 | d2 | d3 | d4 | d5 | d6 | d7 | stop bit]
	reg [3:0] data_buffer_index=0;
	reg [3:0] data_buffer_base=0;
	reg [7:0] data_buffer[0:9]; // data buffer
	reg [7:0] data_tx=0; // data to transmit
	reg [7:0] rx_data=0;
	reg [3:0] rx_bit_count=0;
	reg [11:0] rx_clk_count=0;
	reg state_rx=0;
	reg tx_bit=0;
	
	reg [24:0]cnt =0;
	reg reset_new = 1'b1;
	reg [2:0]reset_cnt = 0;
	
	reg [15:0] test_led = 0;
	wire clk_slow;
	reg clk_operating;
	wire [31:0] ALU_Result;


	assign clk_slow = clk_operating;
	basic_pipeline my_pipeline(.clk(clk_slow), .reset(reset), .result(ALU_Result));
	
	//Counter
	always @(posedge clk_50MHz or negedge reset) begin
		if(!reset) begin
			counter<=0;
			clk_operating <= 1'b0;
		end

		else begin
			counter<=counter+1;
			clk_operating <= ((counter[3:0] == 4'b1111) ? ~clk_operating: clk_operating );
		end
	end

	
	// // Test Led
	// always @(posedge counter[24] or negedge reset) begin
	// 	if (!reset) begin
	// 		led <= 16'b0;
	// 	end

	// 	else begin
	// 		// 7 9 10 ... 23
    //     	led<= ~test_led;
	// 		test_led <= ~test_led;
	// 	end
	// end

	// SLOW Led
	always @(posedge clk_50MHz or negedge reset) begin
		if (!reset) begin
			led <= 16'b0;
		end

		else begin
			led<= ALU_Result[15:0];
		end
	end
	
	//Key & KeyLed
	always @(posedge clk_50MHz or negedge reset) begin
		if(!reset) begin
			keyLed <= 5'b11111;
		end
		
		else begin
			keyLed <= ~key;
		end
	end

	//Fnd number
	always @(posedge counter[10] or negedge reset) begin 
		if(!reset) begin 
			digit_sel<=0;
			digit<=4'b0000;
			number<=0;
		end
		
		else
			begin
				digit_sel<=digit_sel+1'b1;
				if(digit_sel==2'b11)
					digit_sel<=0;
					case(digit_sel)
						0: begin digit<=4'b1110; number<=0; end
						1: begin digit<=4'b1101; number<=1; end
						2: begin digit<=4'b1011; number<=2; end
						3: begin digit<=4'b0111; number<=3; end	
					endcase
		end
	end
	
	//Buzzer
	always @(posedge counter[24] or negedge reset) begin
		if(!reset) begin
			state <= st_Init;
		end
		else begin
			case(state)
			st_Init: begin 
					buzz<=0; 
					alert_num<=0; 
					state<=st_BuzzOn;
					end
			st_BuzzOn: begin 
					//buzz<=1; 
					buzz<=0; 
					alert_num<=alert_num+1; 
					state<=st_BuzzOff;
					end
			st_BuzzOff: begin 
					buzz<=0; 
					alert_num<=alert_num;  
					if(alert_num<4) state<=st_BuzzOn; 
					else state<=st_BuzzOff; 
					end
			default : begin 
					//buzz<=buzz; 
					buzz<=0; 
					alert_num<=alert_num; 
					state<=state;
					end
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
	
	//Uart
	
	always@(posedge clk_50MHz)
	begin
		if(reset_new == 1'b1)
		reset_cnt <= reset_cnt + 1'b1;
		if(reset_cnt == 4'd7)
		reset_new <= 1'b0;
	end 

	always@(posedge clk_50MHz)
	begin
		if(reset_new == 1'b1)
		begin
		cnt <= 32'd0;
		end
		else 
			begin
				if(cnt == 32'd25000000)
				begin
				cnt <= 32'd0;
				end 
				else cnt <= cnt + 1'b1;
			end 
	end 
   
	// Transmitter Process 
	// at every rising edge of the clock
    always @ (posedge clk_50MHz)
    begin
        if(!reset)begin
            tx_clk_count = 0;
            tx_bit_count = 0;
            tx_bit = 1;                     // set idle
            data_buffer_index = 0;          // data index
        end
        else begin
            // transmit data until the index became the same with the base index
            if ( data_buffer_index != data_buffer_base ) begin
                if (tx_clk_count == CLOCKS_PER_BIT) begin
                    if (tx_bit_count == 0) begin
                        tx_bit = 1'b1;     // idle bit
                        tx_bit_count = 1'b1;
                        data_tx = data_buffer[data_buffer_index];
                    end
                    else if (tx_bit_count == 1) begin
                        tx_bit = 0;     // start bit
                        tx_bit_count = 2;
                    end
                    else if (tx_bit_count <= MAX_TX_BIT_COUNT) begin
                        tx_bit = data_tx[tx_bit_count-2];   // data bits
                        tx_bit_count = tx_bit_count + 1'b1;
                    end
                    else begin
                        tx_bit = 1;     // stop bit
								
                        data_buffer_index = data_buffer_index + 1'b1;  // if the index exceeds its maximum, it becomes 0.
                        if(data_buffer_index==4'd10)  ///reset!
								begin
									data_buffer_index=4'd0;
								end 
								tx_bit_count = 0;
                    end
                    tx_clk_count = 0;   // reset clock count
                end
                 
                tx_clk_count = tx_clk_count + 1'b1;        // increase clock count
            end
        end
    end
     
    // Receiver Processs
    // at every rising edge of the clock
    always @ (posedge clk_50MHz)
    begin
        if (reset_new == 1) begin
            rx_clk_count = 0;
            rx_bit_count = 0;
            data_buffer_base = 0;               // base index
            state_rx = 0;
        end
        else begin
            // if not receive mode and start bit is detected
            if (state_rx == 0 && RXD == 0) begin
                state_rx = 1;       // enter receive mode
                rx_bit_count = 0;
                rx_clk_count = 0;
            end
            // if receive mode
            else if (state_rx == 1) begin
                 
                if(rx_bit_count == 0 && rx_clk_count == CLOCKS_WAIT_FOR_RECEIVE) begin
                    rx_bit_count = 1;
                    rx_clk_count = 0;
                end
                else if(rx_bit_count < 9 && rx_clk_count == CLOCKS_PER_BIT) begin
                    rx_data[rx_bit_count-1] = RXD;
                    rx_bit_count = rx_bit_count + 1'b1;
                    rx_clk_count = 0;
                end
                // stop receiving
                else if(rx_bit_count == 9 && rx_clk_count == CLOCKS_PER_BIT && RXD == 1) begin
                    state_rx = 0;
                    rx_clk_count = 0;
                    rx_bit_count = 0;
                     
                    // transmit the received data back to the host PC.
                    data_buffer[data_buffer_base] = rx_data;
                    data_buffer_base = data_buffer_base + 1'b1;        // if the index exceeds its maximum, it becomes 0.
						  if(data_buffer_base == 4'd10)
						  begin
						  data_buffer_base = 4'd0;
						  end 
					 end
                // if stop bit is not received, clear the received data
                else if(rx_bit_count == 9 && rx_clk_count == CLOCKS_PER_BIT && RXD != 1) begin
                    state_rx = 0;
                    rx_clk_count = 0;
                    rx_bit_count = 0;
                    rx_data = 8'b00000000;      // invalidate
                end
                rx_clk_count = rx_clk_count + 1'b1;
            end
        end
         
    end
     
    assign TXD = tx_bit;
endmodule

