`timescale 1ps / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: Giray
// 
// Create Date:    10:20:39 12/15/2016 
// Design Name: 
// Module Name:    short_term_history 
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
// 	Short Time History for the Recently Accessed Rows
// 	There are NUM_BANKS registers each has a row address 
// 	and one extra bit indicating it is open(1) or closed(0)
//////////////////////////////////////////////////////////////////////////////////
`include "softMC.inc"
module bank_states
#(parameter ROW_WIDTH = 15, BANK_WIDTH = 3, CS_WIDTH = 1)
(
	input wire clk, rst,
	
	// Feed from CMD_RECV
	input wire [31:0] instr,
	input wire is_app, is_mnt,
	
	// Request from MAINT_HANDLR
	input wire [BANK_WIDTH - 1 : 0] maint_bank,
	// Reply Message
	output wire [ROW_WIDTH : 0] maint_bank_state
);
	
	wire [BANK_WIDTH-1 : 0] bank_index;
	wire [ROW_WIDTH-1  : 0] row_addr;
	wire is_act, is_pre;
	
	assign bank_index = instr[ROW_WIDTH +: BANK_WIDTH];
	assign row_addr   = instr[ROW_WIDTH - 1 : 0];
	assign is_act = instr[31] && ~|instr[`CS_OFFSET +: CS_WIDTH] && ~instr[`RAS_OFFSET] && instr[`CAS_OFFSET] && instr[`WE_OFFSET];
	assign is_pre = instr[31] && ~|instr[`CS_OFFSET +: CS_WIDTH] && ~instr[`RAS_OFFSET] && instr[`CAS_OFFSET] && ~instr[`WE_OFFSET];
	
	localparam NUM_BANKS = 1 << BANK_WIDTH;
	reg [ROW_WIDTH : 0] last_rows_r [0:NUM_BANKS-1];
	reg [ROW_WIDTH : 0] last_rows_ns;
	always @ (posedge clk) begin
		if (rst) begin
			last_rows_r[0] <= 0;
			last_rows_r[1] <= 0;
			last_rows_r[2] <= 0;
			last_rows_r[3] <= 0;
			last_rows_r[4] <= 0;
			last_rows_r[5] <= 0;
			last_rows_r[6] <= 0;
			last_rows_r[7] <= 0;
		end
		else begin
			last_rows_r[bank_index] <= last_rows_ns;
		end
	end
	
	always @ * begin
		last_rows_ns = last_rows_r[bank_index];
		if (is_app) begin
			if (is_act) begin	// ACT 
				last_rows_ns = {1'b1,row_addr};
			end
			if (is_pre) begin // PRE
				last_rows_ns = {1'b0,row_addr};
			end
		end
	end
	
	assign maint_bank_state = last_rows_r[maint_bank];

	`ifdef SIM
		reg [31:0] instr_r;	
		always @ (instr, is_app, is_mnt) begin
			if (is_app) begin
				$display("[",$time,"] @bank_states: App CMD: %h ",instr);	
				if (instr_r == instr && instr != 32'h40000001)
					$stop;
				instr_r = instr;
			end
			if (is_mnt) begin
				$display("[",$time,"] @bank_states: Mnt CMD: %h ",instr);	
				if (instr_r == instr && instr != 32'h40000001)
					$stop;
				instr_r = instr;
			end
		end
	`endif
endmodule
