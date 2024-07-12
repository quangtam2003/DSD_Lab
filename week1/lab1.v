module lab1(SW,LEDG,LEDR);
	input[17:0] SW;
	output[7:0] LEDG;
	output[17:0] LEDR;
	assign LEDR=SW;
	allgate_DF DUT(.a(SW[0]),.b(SW[1]),.yand(LEDG[6]),.yor(LEDG[5]),.ynot(LEDG[4]),.ynand(LEDG[3]),.ynor(LEDG[2]),.yxor(LEDG[1]),.yxnor(LEDG[0]));
endmodule

module allgate_DF ( a, b, yand,yor,ynot,ynand,ynor,yxor,yxnor );
input a,b;
output yand, yor, ynot, ynand, ynor, yxor, yxnor;

assign yand = a & b;		// AND Operation
assign yor = a | b;		// OR Operation
assign ynot = ~a ;		// NOT Operation
assign ynand = ~(a & b);	// NAND Operation
assign ynor = ~(a | b);		//NOR Operation
assign yxor = a ^ b;		//XOR Operation
assign yxnor =~(a^b);		//XNOR Operation
endmodule				// END of the module