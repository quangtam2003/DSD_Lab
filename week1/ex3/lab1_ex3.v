module lab1_ex3(SW,LEDG,LEDR);
	input[17:0] SW;
	output[7:0] LEDG;
	output[17:0] LEDR;
	assign LEDR=SW;
	full_adder_DF DUT(SW[0],SW[1],SW[2],LEDG[1],LEDG[0]);
	full_adder_BH1 DUT1(SW[3],SW[4],SW[5],LEDG[2],LEDG[3]);
	half_adder_structeral DUT2(SW[6],SW[7],SW[8],LEDG[4],LEDG[5]);
	
endmodule

module full_adder_DF(input a, b, cin, output S, Cout);
  assign S = a ^ b ^ cin;
  assign Cout = (a & b) | (b & cin) | (a & cin);
endmodule

module full_adder_BH1 (input wire A, B, Cin, output reg S, output reg Cout);
 always @(A or B or Cin)
  begin 

   case (A | B | Cin) 
     3'b000: begin S = 0; Cout = 0; end 
     3'b001: begin S = 1; Cout = 0; end 
     3'b010: begin S = 1; Cout = 0; end 
     3'b011: begin S = 0; Cout = 1; end 
     3'b100: begin S = 1; Cout = 0; end 
     3'b101: begin S = 0; Cout = 1; end 
     3'b110: begin S = 0; Cout = 1; end 
     3'b111: begin S = 1; Cout = 1; end 
   endcase 
	end
	endmodule
  
  
  module full_adder_STRU(a,b,cin,Cum,Carry);
	output Sum,Carry;
	input A,B,Cin;
	wire x,y,z;
	xor g1(x,A,B);
            xor g2(Sum,x,Cin);
	and g3(y,x,Cin);
	and g4(z,A,B);
	or  g5(Carry,x,y);
endmodule