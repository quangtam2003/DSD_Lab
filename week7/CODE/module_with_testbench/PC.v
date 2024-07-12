module Program_Counter (clk, reset, PC_in, PC_out);
	input clk, reset;
	input [31:0] PC_in;
	output reg [31:0] PC_out;
	always @ (posedge clk or posedge reset)
	begin
		if(reset==1'b1)
			PC_out<=0;
		else
			PC_out<=PC_in;
	end
endmodule
//Test bench
module Tbench;
    reg clk, reset;
	reg [31:0] PC_in;
	wire [31:0] PC_out;
	Program_Counter dut(clk, reset, PC_in, PC_out);
    always #5 clk=~clk;
    initial begin
        $monitor($time,"clk = %b, reset = %b, PC_in = %2d, PC_out = %2d",clk, reset, PC_in, PC_out);
                clk=0;reset=1;
            #10 reset =0;
            #10 PC_in = 6'b000111;
            #10 PC_in = 6'b010101;
            #10 PC_in = 6'b011111;
            #10 PC_in = 6'b100111;
            #10 $finish;
    end
endmodule