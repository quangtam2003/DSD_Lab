module Mux4_32_bit (in0, in1,in2, in3, mux_out, select);
	parameter N = 32;
	input [N-1:0] in0, in1,in2,in3;
	output [N-1:0] mux_out;
	input [1:0]select;
	assign mux_out = select[1]? (select[0]?in3: in2):(select[0]?in1:in0);
endmodule


module tb;
	parameter N = 32;
	reg [N-1:0] in0, in1,in2,in3;
	wire [N-1:0] mux_out;
	reg [1:0]select;
	
	Mux4_32_bit dut(in0, in1,in2, in3, mux_out, select);
	
	initial begin
	    $monitor($time," in0=%d, in1=%d, in2=%d, in3=%d, mux_out=%d, select=%b",in0, in1,in2, in3, mux_out, select);
	    
	        in2=32'd66; in3=32'd99; in0=32'd11; in1=32'd33; select=0;
            repeat (4) #2 select=select+1;
		#5	$finish;
	end
endmodule