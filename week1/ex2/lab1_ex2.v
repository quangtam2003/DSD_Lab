module lab1_ex2(SW,LEDG,LEDR);
	input[17:0] SW;
	output[7:0] LEDG;
	output[17:0] LEDR;
	assign LEDR=SW;
	half_adder_dataflow DUT(SW[1],SW[2],LEDG[1],LEDG[0]);
endmodule

module half_adder_dataflow(input a, b, output s, Cout);
  assign s = a ^ b;
  assign Cout = a & b;
endmodule

