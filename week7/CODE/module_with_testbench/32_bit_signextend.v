module tb;
	reg 	[15:0] sign_in;
	wire 	[31:0] sign_out;	

	N_Sign_Extension DUT (sign_in, sign_out);

	initial begin
		$monitor ($time, " sign_in=%b, sign_out=%b",sign_in, sign_out);
			sign_in=16'b1000_1000_1111_1100;
		#5 	sign_in=16'b10;
		#5  $finish;
	end
endmodule
module N_Sign_Extension (sign_in, sign_out);
	parameter N=32;
	input 	[15:0] sign_in;
	output  [N-1:0] sign_out;
	assign sign_out[15:0]=sign_in[15:0];
	assign sign_out[N-1:16]=sign_in[15]?16'b1111_1111_1111_1111:16'b0;
endmodule


