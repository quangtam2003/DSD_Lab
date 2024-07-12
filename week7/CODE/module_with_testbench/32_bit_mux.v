module tb;
    parameter N=32;
	reg 	[N-1:0] in0, in1;
	wire 	[N-1:0] mux_out;
	reg select;

	Mux_N_bit DUT (in0, in1, mux_out, select);

	initial begin
		$monitor($time," in0=%d, in1=%d, mux_out=%d, select=%b",in0, in1, mux_out, select);
			in0=32'd66; in1=32'd99; 
		#5	select=1'b0;
		#5	select=1'b1;
		#5	$finish;
	end
endmodule

module Mux_N_bit (in0, in1, mux_out, select);
	parameter N = 32;
	input [N-1:0] in0, in1;
	output [N-1:0] mux_out;
	input select;
	assign mux_out = select? in1: in0 ;
endmodule
