`timescale 1ns / 1ps

`include "softMC.inc"

module instr_receiver (
	input clk,
	input rst,
	
	input dispatcher_ready,
	
	input rdback_fifo_empty,
	
	input app_en,
	output reg app_ack,
	input[31:0] app_instr,
	
	input maint_en,
	output reg maint_ack,
	input[31:0] maint_instr,
	
	output [31:0] issued_instr,
	output is_issued_app_instr,
	output is_issued_mnt_instr,
	
	output instr0_fifo_en,
	output[31:0] instr0_fifo_data,
	input instr0_fifo_full,
	
	output instr1_fifo_en,
	output[31:0] instr1_fifo_data,
	input instr1_fifo_full,
	
	output process_iseq,
	output process_iseq_host
);

reg process_iseq_r = 1'b0, process_iseq_ns;
reg process_iseq_host_r = 1'b0, process_iseq_host_ns;

localparam STATE_IDLE = 2'b00;
localparam STATE_APP = 2'b01;
localparam STATE_MAINT = 2'b10;

reg[1:0] state_ns, state_r;

reg sel_fifo = 1'b0;

reg instr_en_ns, instr_en_r;
reg[31:0] instr_ns, instr_r;

wire is_any_fifo_full = instr0_fifo_full | instr1_fifo_full;

//register rdback_fifo_empty for better timing
reg rdback_fifo_empty_r = 1;
always@(posedge clk) begin
	if(rst) begin
		rdback_fifo_empty_r <= 1;
	end
	else begin
		rdback_fifo_empty_r <= rdback_fifo_empty;
	end
end

always@* begin
	process_iseq_ns = 1'b0;
	process_iseq_host_ns = 1'b0;
	
	state_ns = state_r;
	
	instr_en_ns = 1'b0;
	instr_ns = instr_r;
	
	app_ack = 1'b0;
	maint_ack = 1'b0;
	
	case(state_r)
		STATE_IDLE: begin
			if(dispatcher_ready & ~process_iseq_r & rdback_fifo_empty_r) begin //do not start processing a new Iseq 1) before completing execution of the current Iseq, and 2) transferring the data read from the DRAM to the host machine
				if(maint_en & ~is_any_fifo_full) begin
					state_ns = STATE_MAINT;
					instr_en_ns = maint_en;
					instr_ns = maint_instr;
					
					maint_ack = 1'b1;
				end
				else if(app_en & ~is_any_fifo_full) begin
					state_ns = STATE_APP;
					instr_en_ns = app_en;
					instr_ns = app_instr;
					
					app_ack = 1'b1;
				end
			end //dispatcher_ready
		end //STATE_IDLE
		
		STATE_APP: begin
			if(~is_any_fifo_full) begin
				app_ack = 1'b1;
				
				instr_en_ns = app_en;
				instr_ns = app_instr;
				
				if(instr_en_ns & (instr_ns[31:28] == `END_ISEQ)) begin
					process_iseq_ns = 1'b1;
					process_iseq_host_ns = 1'b1;
					state_ns = STATE_IDLE;
				end
			end //~is_any_fifo_full
		end //STATE_APP
		
		STATE_MAINT: begin
			maint_ack = 1'b1;
			
			instr_en_ns = maint_en;
			instr_ns = maint_instr;
			
			if(instr_en_ns & (instr_ns[31:28] == `END_ISEQ)) begin
				instr_en_ns = 1'b0;
				process_iseq_ns = 1'b1;
				state_ns = STATE_IDLE;
			end
		end //STATE_MAINT
	
	endcase //state_r
end //always

assign instr0_fifo_en = ~sel_fifo & instr_en_r;
assign instr0_fifo_data = instr_r;
assign instr1_fifo_en = sel_fifo & instr_en_r;
assign instr1_fifo_data = instr_r;

assign issued_instr = instr_r;
assign is_issued_app_instr = instr_en_r & (state_r == STATE_APP);
assign is_issued_mnt_instr = instr_en_r & (state_r == STATE_MAINT);

always@(posedge clk) begin
	if(rst) begin
		process_iseq_r <= 1'b0;
		sel_fifo <= 1'b0;
		state_r <= STATE_IDLE; 
		
		instr_en_r <= 1'b0;
		instr_r <= 0;
	end
	else begin
		state_r <= state_ns;
		process_iseq_r <= process_iseq_ns;
		process_iseq_host_r <= process_iseq_host_ns;
		
		instr_en_r <= instr_en_ns;
		instr_r <= instr_ns;
		
		if(process_iseq_r)
			sel_fifo <= 1'b0;
		else if(instr_en_r)
			sel_fifo <= ~sel_fifo;
	end //!rst
end

assign process_iseq = process_iseq_r;
assign process_iseq_host = process_iseq_host_r;

endmodule
