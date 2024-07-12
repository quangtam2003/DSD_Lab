module tb;
    reg [2:0]alufn;
    reg [31:0] ra;
    reg [31:0] rb_or_imm;
    wire [31:0]aluout;
    wire zero;
    alu DUT(alufn,ra,rb_or_imm,aluout,zero);
    always #5 alufn=alufn+1;
    initial begin
        $monitor($time  ," alufn = %b ra=%d rb_or_imm=%d| aluout = %d zero=%b",alufn,ra,rb_or_imm,aluout,zero);
            ra=32'd8;rb_or_imm=32'd2;alufn=0;
        #36 ra=32'd8;rb_or_imm=32'd8;
       
        #1 $finish;
        end
endmodule

module alu(
	input [2:0] alufn,
	input [31:0] ra,
	input [31:0] rb_or_imm,
	output reg [31:0] aluout,
	output reg zero);
	parameter	ALU_OP_ADD	    = 3'b000,
			    ALU_OP_SUB	    = 3'b001,
			    ALU_OP_AND	    = 3'b010,
			    ALU_OP_OR		= 3'b011,
			    ALU_OP_XOR	    = 3'b100,
			    ALU_OP_LW		= 3'b101,
			    ALU_OP_SW		= 3'b110,
			    ALU_OP_BEQ	    = 3'b111;

	always @(*) 
	begin
			case(alufn)
				ALU_OP_ADD 	    : aluout = ra + rb_or_imm;
				ALU_OP_SUB 	    : aluout = ra - rb_or_imm;
				ALU_OP_AND 	    : aluout = ra & rb_or_imm;
				ALU_OP_OR	    : aluout = ra | rb_or_imm;
				ALU_OP_XOR	    : aluout = ra ^ rb_or_imm;
				ALU_OP_LW	    : aluout = ra + rb_or_imm;
				ALU_OP_SW	    : aluout = ra + rb_or_imm;
				ALU_OP_BEQ	    : begin
						            zero = (ra==rb_or_imm)?1'b1:1'b0;
						            aluout = ra - rb_or_imm;
						          end
			endcase
	end
endmodule

