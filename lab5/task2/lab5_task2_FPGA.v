module lab5_task2_FPGA(SW,LEDR,LEDG,KEY,HEX7, HEX6, HEX5, HEX4, HEX3, HEX2, HEX1, HEX0);
	input [17:0] SW;
    input [3:0] KEY;
    output [0:6] HEX7, HEX6, HEX5, HEX4, HEX3, HEX2, HEX1, HEX0;
	output [17:0] LEDR;
	output [7:0] LEDG;
	wire [7:0] W_RD1,W_RD2,ALU_out;
	assign LEDR=SW;
	hex_ssd C1(ALU_out[3:0], HEX0);
	hex_ssd C2(ALU_out[7:4], HEX1);
	hex_ssd C3(W_RD1[3:0], HEX2);
	hex_ssd C4(W_RD1[7:4], HEX3);
	hex_ssd C5(W_RD2[3:0], HEX4);
	hex_ssd C6(W_RD2[7:4], HEX5);	
	Datapath_R_Type DUT(.clk(KEY[0]),.reset(KEY[2]),.RegWrite(KEY[1]), .rs(SW[14:10]),.rt(SW[9:5]),.rd(SW[4:0]),.ALUop(SW[17:15]),.Zero(LEDG[0]),.W_RD1(W_RD1),.W_RD2(W_RD2),.ALU_out(ALU_out));
//	Datapath_R_Type DUT(.clk(1/0),.reset(1/0),.RegWrite(1/0), .rs(17),.rt(18),.rd(16),.ALUop(000),.Zero(0),.W_RD1(h66),.W_RD2(h33),.ALU_out(h99));
endmodule



module Datapath_R_Type(clk,reset,RegWrite, rs, rt, rd,ALUop,Zero,W_RD1,W_RD2,ALU_out);
input clk,reset,RegWrite;
input [4:0] rs, rt, rd;
input [2:0] ALUop;
output Zero;
output [7:0] ALU_out;
output  [7:0] W_RD1,W_RD2;
Register_File C1(.clk(clk), .reset(reset),.RegWrite(RegWrite),
.write_data(ALU_out),.read_addr_1(rs), .read_addr_2(rt), .write_addr(rd), 
.read_data_1(W_RD1), .read_data_2(W_RD2));

alu C2(.alufn(ALUop),.ra(W_RD1),.rb_or_imm(W_RD2),.aluout(ALU_out),.zero(Zero));

endmodule

module Register_File (clk, reset,RegWrite,write_data,read_addr_1, read_addr_2, write_addr, read_data_1, read_data_2);
	input [4:0] read_addr_1, read_addr_2, write_addr;
	input [7:0] write_data;
	input clk, reset, RegWrite;
	output [7:0] read_data_1, read_data_2;

	reg [7:0] Regfile [31:0];
	integer k;
	assign read_data_1 = Regfile[read_addr_1];

	assign read_data_2 = Regfile[read_addr_2];

    always @(negedge clk or negedge reset) 
	  begin
		if (reset==1'b0)
		begin
			for (k=0; k<32; k=k+1) 
			begin
				Regfile[k] = k;
			end
			Regfile[17]=8'h66;
			Regfile[18]=8'h33;
			
			Regfile[10]=8'h55;
			Regfile[12]=8'h22;
			
			Regfile[2]=8'h09;
			Regfile[6]=8'h02;
			
			
		end
		else if (RegWrite == 1'b0) begin    
		         Regfile[write_addr] = write_data; 
			     $display("write data",Regfile[write_addr]);
		end
	 end
endmodule


module alu(
	input [2:0] alufn,
	input [7:0] ra,
	input [7:0] rb_or_imm,
	output reg [7:0] aluout,
	output reg zero);

parameter	ALU_OP_ADD	= 3'b000,
			ALU_OP_SUB	= 3'b001,
			ALU_OP_AND	= 3'b010,
			ALU_OP_OR		= 3'b011,
			ALU_OP_NOT_A	= 3'b100,
			ALU_OP_LW		= 3'b101,
			ALU_OP_SW		= 3'b110,
			ALU_OP_BEQ	= 3'b111;
always @(*) 
begin
		case(alufn)
			ALU_OP_ADD 	: aluout = ra + rb_or_imm;
			ALU_OP_SUB 	: aluout = ra - rb_or_imm;
			ALU_OP_AND 	: aluout = ra & rb_or_imm;
			ALU_OP_OR		: aluout = ra | rb_or_imm;
			ALU_OP_NOT_A	: aluout = ~ ra;
			ALU_OP_LW		: aluout = ra + rb_or_imm;
			ALU_OP_SW		: aluout = ra + rb_or_imm;
			ALU_OP_BEQ	: begin
						    zero = (ra==rb_or_imm)?1'b1:1'b0; 
						    aluout = ra - rb_or_imm;
						  end
		endcase
end

endmodule

module hex_ssd (BIN, SSD);
  input [3:0] BIN;
  output reg [0:6] SSD;

  always@(*) begin
    case(BIN)
      0:SSD=7'b0000001;
      1:SSD=7'b1001111;
      2:SSD=7'b0010010;
      3:SSD=7'b0000110;
      4:SSD=7'b1001100;
      5:SSD=7'b0100100;
      6:SSD=7'b0100000;
      7:SSD=7'b0001111;
      8:SSD=7'b0000000;
      9:SSD=7'b0001100;
      10:SSD=7'b0001000;
      11:SSD=7'b1100000;
      12:SSD=7'b0110001;
      13:SSD=7'b1000010;
      14:SSD=7'b0110000;
      15:SSD=7'b0111000;
    endcase
  end
endmodule