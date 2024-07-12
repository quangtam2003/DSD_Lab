module Program_Counter (clk, reset, PC_in, PC_out);
	input clk, reset;
	input [7:0] PC_in;
	output [7:0] PC_out;
	reg [7:0] PC_out;
	always @ (posedge clk or posedge reset)
	begin
		if(reset==1'b1)
			PC_out<=8'b0;
		else
			PC_out<=PC_in;
	end
endmodule

module lab4_task1(SW,LEDR,LEDG,KEY);
	input [17:0] SW;
   input [3:0] KEY;
	output [17:0] LEDG;
	output LEDR;
	assign LEDR=SW;
	Program_Counter (.clk(KEY[0]), .reset(SW[17]), .PC_in(SW[7:0]), .PC_out(LEDG[7:0]));
endmodule