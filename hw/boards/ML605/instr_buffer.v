`timescale 1ps / 1ps

`include "softMC.inc"
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date:    17:43:41 09/27/2016 
// Design Name: 
// Module Name:    instr_buffer 
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
// The buffer stores the whole application. Initial state is IDLE where the buffer 
// is completely empty. Once the @phy_top starts sending commands, @instr_buffer 
// switches to the state FILL and buffers the whole application. `SEND_TR command 
// indicates the application is transferred completely and makes @instr_buffer to 
// switch to PUSH state. @instr_buffer stays in the PUSH state continuously and mimics 
// @phy_top by sending the commands to the @instr_recv. Once the application reachs to 
// the last command @instr_buffer resets the program_counter and continues sending the
// commands. 
//
// TODO: DONE state might be added for Moore machine
// 
// Warning: 
// @instr_recv starts incoming commands to the @instr_fifos only if @tr_dispatcher is
// ready. @tr_dispatcher's ready signal is driven by the empty output of @instr_fifos.
// Implementing this module, we do not want fifos to get empty and full all the time. 
// Instead keeping the fifos utilized everytime is preferred to avoid any bubbles in 
// the execution time. Therefore, some of the control signals in safariMC module has 
// been changed. To see all the changes please refer to the infinite_loop branch on 
// the git repository. 
// 
//
//////////////////////////////////////////////////////////////////////////////////

module instr_buffer(
	input wire clk,
	input wire rst,
	input wire loop_en,
	input wire app_en,
	input wire fifo_ready,
	input wire process_tr,
	input wire dispatcher_ready,
	input wire app_ack_in,
	input wire [31:0] app_instr_in,
//	input wire [1:0] instr_recv_state,
	
	output wire [31:0] app_instr_out,
	output wire app_instr_ready,
	output wire fake_dispatcher_ready,
	output wire app_ack,
//	output wire first_loop,
//	output wire end_of_loop,
//	output wire [10:0] read_max_addr,
//	output wire loop_flag,
//	output wire [2:0] state,
	output wire looping
    );

	localparam 	BRAM_MAX_ADDR	 	= 12'd2047,
					ADDR_SIZE  		 	= 12,  
					CMD_SIZE   		 	= 32, 
					INIT_SIZE		 	= 12'd512,
					STATE_IDLE 		 	= 3'b000, 
					STATE_FILL 		 	= 3'b001,
					STATE_SENDTR 	 	= 3'b010,
					STATE_INF_LOOP	 	= 3'b011,
					STATE_LAST_LOOP 	= 3'b100,
					STATE_LAST_SENDTR = 3'b101;

	reg [ 2:0] state_r, state_ns;
	wire stop_detected, send_tr_detected;
	
	wire [ CMD_SIZE-1  : 0 ] bram_data_out_a, bram_data_out_b, bram_data_in_a;
	reg  [ ADDR_SIZE-1 : 0 ] bram_addr_in_a , bram_addr_in_b;
	
	wire [ ADDR_SIZE-1 : 0 ] bram_wr_addr, bram_rd_addr_a, bram_rd_addr_b;
	wire bram_full, last_in_init, end_of_loop, instr_sel, buffer_reset;
	reg  bram_wr_en;
	
	///////////////////////////////////////////
	// State Machine
	// IDLE ------> FILL -------> STARTTR ------> PUSH
	//  00  app_en   01  SEND_TR    11             10
	///////////////////////////////////////////
	reg dispatcher_ready_r;
	always @ (posedge clk) dispatcher_ready_r <= dispatcher_ready;
	
	always @ (posedge clk) begin
		if (rst) begin
			state_r <= STATE_IDLE;
		end
		else begin
			state_r <= state_ns;
		end
	end

	always @ * begin
		state_ns = state_r;
		case (state_r)
			STATE_IDLE: begin
				if(dispatcher_ready_r & ~process_tr & fifo_ready) begin
					if ( app_en ) begin
						state_ns = STATE_FILL;
					end
				end
			end
			STATE_FILL: begin
				if (send_tr_detected) begin
					state_ns = STATE_SENDTR;
				end
			end
			STATE_SENDTR : begin
				state_ns = STATE_INF_LOOP;
			end
			STATE_INF_LOOP : begin
				if (stop_detected) begin
					state_ns = STATE_LAST_LOOP;
				end
				if (end_of_loop & last_in_init) begin
					state_ns = STATE_LAST_SENDTR;
				end
			end
			STATE_LAST_LOOP: begin
				if (end_of_loop) begin
					state_ns = STATE_LAST_SENDTR;
				end
			end
			STATE_LAST_SENDTR: begin
				if (buffer_reset) begin
					state_ns = STATE_IDLE;
				end
			end
		endcase
	end
	//assign first_loop = (state_r == STATE_IDLE) | (state_r == STATE_FILL);
	assign stop_detected = (app_instr_in[31:28] == `STOP && app_en);	
	assign send_tr_detected = (app_instr_in[31:28] == `END_ISEQ && app_en);
	assign looping = (state_r == STATE_INF_LOOP)
						| (state_r == STATE_LAST_LOOP)
						| (state_r == STATE_LAST_SENDTR);
//	assign state = state_r;

	///////////////////////////////////////////
	// Block RAM I/O Logic
	///////////////////////////////////////////
	instr_buffer_bram i_instr_buffer_bram (
	  .clka(clk), 					// input  clka
	  .rsta(rst), 					// input  rsta
	  .wea(bram_wr_en), 			// input  [0 : 0] wea
	  .addra(bram_addr_in_a), 	// input  [10 : 0] addra
	  .dina(bram_data_in_a), 	// input  [31 : 0] dina
	  .douta(bram_data_out_a), // output [31 : 0] douta
	  .clkb(clk), 					// input  clkb
	  .web(1'b0), 					// input  [0 : 0] web
	  .addrb(bram_addr_in_b), 	// input  [10 : 0] addrb
	  .dinb(32'd0), 				// input  [31 : 0] dinb
	  .doutb(bram_data_out_b) 	// output [31 : 0] doutb
	);
	
	always @ * begin
		bram_wr_en = 1'b0;
		bram_addr_in_a = bram_rd_addr_a;
		bram_addr_in_b = bram_rd_addr_b;
		case (state_r) 
			STATE_FILL: begin
				bram_wr_en = ~bram_full && app_en;
				bram_addr_in_a = bram_wr_addr;
			end
		endcase
	end

	assign bram_data_in_a = app_instr_in;
	assign app_instr_out = (instr_sel)? bram_data_out_b : bram_data_out_a;
	
	///////////////////////////////////////////
	// Counter Logic
	///////////////////////////////////////////
	buffer_bram_controller #(
			.WIDTH(ADDR_SIZE), 
			.MAX_ADDR(BRAM_MAX_ADDR),
			.LOOP_START(INIT_SIZE)
	)
	i_bram_controller (
		.clk(clk), .rst(rst), 
		.wr_en(bram_wr_en), .app_en(app_en) ,
		.wr_addr(bram_wr_addr), 
		.full(bram_full), .last_in_init(last_in_init),
		.last_loop(state_r == STATE_LAST_SENDTR | state_r == STATE_LAST_LOOP), 
		.loop_en(loop_en), .app_ack(app_ack_in), 
		.fifo_ready(fifo_ready), .looping(looping), 
		.rd_addr_a(bram_rd_addr_a), .rd_addr_b(bram_rd_addr_b), 
		.end_of_loop(end_of_loop),
		.app_en_o(app_instr_ready),
		.instr_sel(instr_sel),
		.buffer_reset(buffer_reset)
	);

	// Output logic
	assign fake_dispatcher_ready = fifo_ready;
	assign app_ack = app_en & (state_r == STATE_INF_LOOP | state_r == STATE_FILL);
endmodule

module instr_buffer_wrapper (
	input wire clk, rst,
			
	input wire fifo_full,
			
	input wire app_en_in,
	input wire app_ack_in,
	input wire [31:0] app_instr_in, 
	
	input wire dispatcher_busy,
	input wire process_tr,
			
	output wire app_en_out,
	output wire app_ack_out,
	output wire [31:0] app_instr_out, 
	
	output wire dispatcher_ready,
	output wire looping
);

`ifdef LOOP_EN
	wire fifo_ready;
	wire dispatcher_ready_buff_out, app_en_buff_out, app_ack_buff_out;
	wire [31:0] app_instr_buff_out;
	
	// To match timing constraints looping signal is latched
	wire looping_ns; reg looping_r;
	always @ (posedge clk) looping_r <= looping_ns;
	assign looping = looping_r;

	assign fifo_ready = ~fifo_full;
	instr_buffer i_instr_buffer (
		 .clk(clk), 
		 .rst(rst), 
		 .loop_en(1'b1), 
		 .app_en(app_en_in), 
		 .process_tr(process_tr),
		 .fifo_ready(fifo_ready), 
		 .dispatcher_ready(~dispatcher_busy),
		 .app_instr_in(app_instr_in),
		 .app_ack_in(app_ack_in),
		 .app_instr_out(app_instr_buff_out), 
		 .app_instr_ready(app_en_buff_out), 
		 .fake_dispatcher_ready(dispatcher_ready_buff_out), 
		 .app_ack(app_ack_buff_out),
		 .looping(looping_ns)
		 );
`endif

// Output Logic
`ifdef LOOP_EN
//	assign buffer_state 		= first_loop;
	assign app_en_out   		= app_en_buff_out;  				//first_loop ?  app_en_in   		: app_en_buff_out;
	assign app_instr_out  		= app_instr_buff_out; 				//first_loop ?  app_instr_in  		: app_instr_buff_out; 
	assign dispatcher_ready = dispatcher_ready_buff_out; 	//first_loop ? ~dispatcher_busy  : dispatcher_ready_buff_out;
	assign app_ack_out		= app_ack_buff_out; 				//first_loop ?  app_ack_in		   : app_ack_buff_out;
`else
	assign app_en_out   		=  app_en_in   	;
	assign app_instr_out  		=  app_instr_in  	; 
	assign dispatcher_ready = ~dispatcher_busy;
	assign app_ack_out		=  app_ack_in		;
	assign looping 			=  1'b0				;
`endif
	endmodule
	
