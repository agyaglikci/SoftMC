`timescale 1ps / 1ps

`include "softMC.inc"
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date:    21:39:48 12/07/2016 
// Design Name: 
// Module Name:    buffer_counters 
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
module buffer_bram_controller( 
	clk, rst, 							// sync input
	wr_en, app_en, 					// write controller input
	wr_addr, full, last_in_init, 	// write controller output
	last_loop, loop_en, app_ack, fifo_ready, looping,  	// read cntrl input
	app_en_o, rd_addr_a, rd_addr_b, end_of_loop, instr_sel, buffer_reset  // read cntrl output 
	);
	
	parameter WIDTH = 11, MAX_ADDR = 2047, LOOP_START = 512;
	
	input  wire clk, rst, last_loop, wr_en, app_en, loop_en;
	input  wire app_ack, fifo_ready, looping;
	output wire full, end_of_loop, app_en_o, last_in_init, instr_sel;
	output wire [WIDTH-1:0] wr_addr, rd_addr_a, rd_addr_b;
	output reg  buffer_reset;
	wire rd_buffer_reset;
	assign last_in_init = wr_addr < LOOP_START;
	
	read_addr_controller 
	#(.WIDTH(WIDTH), .LOOP_START(LOOP_START)) 
	i_rd_addr_cntr (
		.clk(clk), 
		.rst(rst), 
		.last_loop(last_loop), 
		.loop_en(loop_en), 
		.app_ack(app_ack), 
		.fifo_ready(fifo_ready), 
		.looping(looping),
		.max_addr(wr_addr-11'd1), 
		.app_en_o(app_en_o), 
		.addra_o(rd_addr_a), .addrb_o(rd_addr_b), 
		.buffer_reset(rd_buffer_reset),
		.end_of_loop(end_of_loop),
		.instr_sel(instr_sel)
    );
	
	// Signal is latched to match timing constraints
	always @ (posedge clk) 
		buffer_reset <= (rst)? 1'b0 : rd_buffer_reset;
	
	write_addr_counter 
	#(.WIDTH(WIDTH), .MAX_ADDR(MAX_ADDR)) 
	i_wr_addr_cntr (
		.clk(clk), 
		.rst(rst), 
		.zero(buffer_reset), 
		.incr(wr_en & app_en), 
		.full(full),		
		.counter_o(wr_addr)
	);

endmodule 

module write_addr_counter( clk, rst, zero, incr, counter_o, full);
	
	parameter WIDTH = 11, MAX_ADDR = 2047;
	input wire clk, rst, zero, incr;
	output wire full;
	output wire [WIDTH-1:0] counter_o;
	
	reg [WIDTH-1 : 0] counter_r, counter_ns;
	
	always @ (posedge clk) begin
		if (rst) 
			counter_r <= 0;
		else
			counter_r <= counter_ns;
	end
	
	always @ * begin
		casex({zero,incr})
			2'b00: counter_ns = counter_r;
			2'b01: counter_ns = counter_r + 1;
			2'b1x: counter_ns = 0;
		endcase
	end
	
	assign full  = counter_r == MAX_ADDR;
	assign counter_o = counter_r;
	
endmodule

module read_addr_controller(clk, rst, last_loop, loop_en,
	app_ack, fifo_ready, looping, max_addr,
	app_en_o, addra_o, addrb_o, end_of_loop, buffer_reset, instr_sel);
	
	parameter WIDTH = 11, LOOP_START = 512;
	
	input wire clk, rst, last_loop, loop_en;
	input wire app_ack, fifo_ready, looping;
	input wire [WIDTH-1:0] max_addr;
	
	output wire end_of_loop, instr_sel;
	output reg  app_en_o;
	output wire [WIDTH-1:0] addra_o, addrb_o;
	output wire buffer_reset;
	
	localparam IDLE = 2'b00, READA = 2'b10, READB = 2'b11, RESET = 2'b01; //states
	localparam RESET_VAL_A = 11'd0, RESET_VAL_B = 11'd1;
	
	reg  [1:0] state_r, state_ns;
	reg  [WIDTH-1:0] addra_r, addrb_r;
	wire [WIDTH-1:0] addra_ns, addrb_ns;
	reg incr_a, incr_b;
	
	// FFs for the state machine and the counter 
	always @ (posedge clk) begin
		if (rst) begin
			state_r <= IDLE;
			addra_r <= RESET_VAL_A;
			addrb_r <= RESET_VAL_B;
		end
		else begin
			state_r <= state_ns;
			addra_r <= addra_ns;
			addrb_r <= addrb_ns;
		end
	end
	
	// Combinational Logic for Next State
	always @ * begin
		state_ns = state_r;
		incr_a = 1'b0;
		incr_b = 1'b0;
		app_en_o = 1'b0;
		case (state_r)
			IDLE: begin
				if (looping) begin
					state_ns = READA;
				end
			end
			
			READA: begin
				if (fifo_ready & looping) begin
					app_en_o = 1'b1;
					if (app_ack) begin
						incr_a = 1'b1;
						state_ns = READB;
						if (buffer_reset) begin
							state_ns = RESET;
						end
					end
				end
			end
			
			READB: begin
				if (fifo_ready) begin
					app_en_o = 1'b1;
					if (app_ack) begin
						incr_b = 1'b1;
						state_ns = READA;
						if (buffer_reset) begin
							state_ns = RESET;
						end
					end
				end
			end
			
			RESET : begin
				state_ns = IDLE;
			end
		endcase
		
	end
	
	// Combinational Logic for Next Addresses
	incr_comb 
	#(.WIDTH(WIDTH), .LOOP_START(LOOP_START), .RESET_VAL(RESET_VAL_A)) 
	i_incr_comb_a (
		.cntr(addra_r), .cntr_ns(addra_ns), .max_cnt(max_addr), 
		.loop_en(loop_en), .last_loop(last_loop), 
		.incr(incr_a), .end_of_code(end_of_code_a),  
		.buffer_reset(buffer_reset_a)
	);
	 
	incr_comb 
	#(.WIDTH(WIDTH), .LOOP_START(LOOP_START), .RESET_VAL(RESET_VAL_B)) 
	i_incr_comb_b (
		.cntr(addrb_r), .cntr_ns(addrb_ns), .max_cnt(max_addr), 
		.loop_en(loop_en), .last_loop(last_loop),
		.incr(incr_b), .end_of_code(end_of_code_b),  
		.buffer_reset(buffer_reset_b)
	);
	
	// Output Logic
	assign end_of_loop  = end_of_code_a  | end_of_code_b;
	assign buffer_reset = buffer_reset_a | buffer_reset_b;
	assign addra_o = addra_r;
	assign addrb_o = addrb_r;
	assign instr_sel = (state_r == READB);
	
endmodule

// @brief: this module is a pure combinational logic
// which defines the counter behavior of read controller.
// Two of these counters drive the block ram ports simultaneously
// for the performance concerns.
module incr_comb 
(
  cntr, cntr_ns, max_cnt, incr, end_of_code,
  loop_en, last_loop, buffer_reset
);

parameter WIDTH = 11, LOOP_START = 512, RESET_VAL = 11'd0;

input wire [WIDTH-1:0] cntr, max_cnt;
output reg [WIDTH-1:0] cntr_ns;

input wire loop_en, last_loop, incr;
output wire end_of_code;
output reg buffer_reset;

wire almost_end_of_code, in_loop;

always @ * begin
	cntr_ns = cntr;
	buffer_reset = 1'b0;

	if (incr) begin
		if (end_of_code | almost_end_of_code) begin
			if (loop_en & ~last_loop & in_loop) begin // loop back
				cntr_ns = (end_of_code)? LOOP_START + 1 : LOOP_START;
			end
			else begin 
				// reached to the end of the last loop => reset
				cntr_ns = RESET_VAL;
				buffer_reset = end_of_code;
			end
		end
		else begin	//increment
			cntr_ns = cntr + 2;
		end
	end

end

assign end_of_code = cntr == max_cnt;
assign almost_end_of_code = cntr == max_cnt - 1;
assign in_loop = cntr >= LOOP_START;

endmodule
