//Test bench
module finalLab(SW,LEDR,LEDG,KEY, HEX5, HEX4, HEX3, HEX2, HEX1, HEX0);
	input [17:0] SW;
    input [3:0] KEY;
    output [0:6] HEX5, HEX4, HEX3, HEX2, HEX1, HEX0;
	output [17:0] LEDR;
	output [7:0] LEDG;

	reg reset, clk;
    wire [31:0] W_PC_out, instruction, W_RD1, W_RD2, W_m1, W_m2, W_ALUout, W_MemtoReg;
    wire Jump;
    	
    assign LEDR=W_PC_out[17:0];
	hex_ssd C1(W_ALUout[3:0], HEX0);
	hex_ssd C2(W_ALUout[7:4], HEX1);
	hex_ssd C3(W_RD1[3:0], HEX2);
	hex_ssd C4(W_RD1[7:4], HEX3);
	hex_ssd C5(W_m1[3:0], HEX4);
	hex_ssd C6(W_m1[7:4], HEX5);	
	
	R_Type_Proc DUT(.reset(SW[17]), .clk(KEY[0]), .W_PC_out(W_PC_out), .instruction(instruction), .W_RD1(W_RD1), .W_RD2(W_RD2), .W_m1(W_m1), .W_m2(W_m2), .W_ALUout(W_ALUout), .W_MemtoReg(W_MemtoReg),.Jump(Jump));	
endmodule



module R_Type_Proc(reset, clk, W_PC_out, instruction, W_RD1, W_RD2, W_m1, W_m2, W_ALUout, W_MemtoReg,Jump);
    input reset, clk;
    output [31:0] W_PC_out, instruction, W_RD1, W_RD2, W_m1, W_m2, W_ALUout, W_MemtoReg;
    output Jump;
    wire [31:0] W_PC_in, W_PC_out, W_PC_plus_1, W_Branch_add, W_RDm,W4,W_Jump_PC;
    wire [4:0] W_m3;
    wire [2:0] ALUop;
    wire PC_Src, RegDst, RegWrite, ALU_Src, zero, MemRead, MemWrite, MemtoReg;
    Program_Counter C1(clk, reset, W_PC_in, W_PC_out);
    Adder32Bit C2(W_PC_out, 32'b1, W_PC_plus_1);
    Mux_32_bit C3(W_PC_plus_1, W_m2, W4, PC_Src);
    Adder32Bit C4(W_PC_plus_1, W_Branch_add, W_m2);
    Instruction_Memory C5(W_PC_out, instruction, reset);
    Mux_5_bit C13(instruction[20:16], instruction[15:11], W_m3, RegDst);
    Register_File C6(clk,instruction[25:21], instruction[20:16], W_m3, W_RD1, W_RD2, W_MemtoReg, RegWrite);
    Sign_Extension C7(instruction[15:0], W_Branch_add);
    Mux_32_bit C8(W_RD2, W_Branch_add, W_m1, ALU_Src);
    alu C9(ALUop, W_RD1, W_m1, W_ALUout, zero);
    Data_Memory C10(W_ALUout, W_RD2, W_RDm, MemRead, MemWrite);
    Mux_32_bit C11(W_ALUout, W_RDm, W_MemtoReg, MemtoReg);
    //Control C12(clk, instruction[31:26],instruction[5:0], RegDst, PC_Src, MemRead, MemtoReg, ALUop, MemWrite, ALU_Src, RegWrite, zero,Jump);
	Control C12(.clk(clk),.Op_intstruct(instruction[31:26]), .ints_function(instruction[5:0]), .RegDst(RegDst), .PcScrc(PC_Src), .MemRead(MemRead), .MemtoReg(MemtoReg), .ALUOp(ALUop), .MemWrite(MemWrite), .ALUSrc(ALU_Src), .RegWrite(RegWrite) , .Zero(zero), .Jump(Jump));
////
//module Control(.clk(clk),.Op_intstruct(instruction[31:26]), .ints_function(instruction[5:0]), .RegDst(RegDst), .PcScrc(PC_Src), .MemRead(MemRead), .MemtoReg(MemtoReg), .ALUOp(ALUop), .MemWrite(MemWrite), .ALUSrc(ALU_Src), .RegWrite(RegWrite) , .Zero(zero), .Jump(Jump));
	//input clk,Zero;
    //input [5:0] ints_function;
    //input [5:0] Op_intstruct;
    //output reg RegDst,PcScrc,MemRead,MemtoReg,Jump;
    //output reg [2:0] ALUOp;
    //output reg MemWrite,ALUSrc,RegWrite;

    
    
    
    Sign_Extension_26   comp14(instruction[25:0], W_Jump_PC);
    Mux_32_bit          comp15(W4, W_Jump_PC, W_PC_in, Jump);
    
endmodule

module Control(clk,Op_intstruct,ints_function,RegDst,PcScrc,MemRead,MemtoReg,ALUOp,MemWrite,ALUSrc,RegWrite,Zero,Jump);
    input clk,Zero;
    input [5:0] ints_function;
    input [5:0] Op_intstruct;
    output reg RegDst,PcScrc,MemRead,MemtoReg,Jump;
    output reg [2:0] ALUOp;
    output reg MemWrite,ALUSrc,RegWrite;
    always @(posedge clk)
    begin
        case(Op_intstruct)
            6'b000000:   // R -Type Instruction
                begin
                    RegDst=1;
                    Jump=0;
                    PcScrc=0;
                    MemRead=0;
                    MemtoReg=0;
                    MemWrite=0;
                    ALUSrc=0;
                    RegWrite=1;
                    ALUOp =3'b000;
                    if(ints_function==6'b100000) // add
                            ALUOp =3'b000;
                                  
                    if(ints_function==6'b100010) // sub 
                            ALUOp =3'b001;
                                   
                    if(ints_function==6'b100100) // and
                            ALUOp =3'b010;
                                   
                    if(ints_function==6'b100101) // or
                            ALUOp =3'b010;
                end
            6'b000100: // beq instruction
                begin
                    RegDst=0;
                    Jump=0;
                    if (Zero==1) begin
                                    PcScrc=1;
                                end 
                    else        begin
                                    PcScrc=0;
                                end
                    MemRead=0;
                    MemtoReg=0;
                    ALUOp =3'b111;
                    MemWrite=0;
                    ALUSrc=0;
                    RegWrite=0;
                  //  $display("beq");
                end
            
            6'b100011:  //load word instruction
                begin
                    RegDst=0;
                    Jump=0;
                    PcScrc=0;
                    MemRead=1;
                    MemtoReg=1;
                    ALUOp =3'b000;
                    MemWrite=0;
                    ALUSrc=1;
                    RegWrite=1;
                  //    $display("LW");
                end
            
            6'b101011: // Store Instruction
                begin
                    RegDst=0;
                    PcScrc=0;
                    Jump=0;
                    MemRead=0;
                    MemtoReg=0;
                    ALUOp =3'b000;
                    MemWrite=1;
                    ALUSrc=1;
                    RegWrite=0;
                  //  $display("SW");
                end
             6'b001000: // addi Instruction
                begin
                    RegDst=0;
                    Jump=0;
                    PcScrc=0;
                    MemRead=0;
                    MemtoReg=0;
                    ALUOp =3'b000;
                    MemWrite=0;
                    ALUSrc=1;
                    RegWrite=1;
                  //    $display("addi");
                end
                
            6'b100010: // jump Instruction
                begin
                    RegDst=0;
                    Jump=1;
                    PcScrc=0;
                    MemRead=0;
                    MemtoReg=0;
                    ALUOp =3'b000;
                    MemWrite=0;
                    ALUSrc=1;
                    RegWrite=1;
                 //   $display("Jump");
                end
            6'b000101:  //bne
                begin
                    RegDst=0;
                    Jump=0;
                    if (Zero==0) begin
                                    PcScrc=1;
                                end 
                    else        begin
                                    PcScrc=0;
                                end
                    MemRead=0;
                    MemtoReg=0;
                    ALUOp =3'b111;
                    MemWrite=0;
                    ALUSrc=0;
                    RegWrite=0;
                  //  $display("brne");
                end
                
            default :
                begin
                    RegDst=1;
                    PcScrc=0;
                    Jump=0;
                    MemRead=0;
                    MemRead=0;
                    MemtoReg=0;
                    ALUOp =3'b000;
                    MemWrite=0;
                    ALUSrc=0;
                    RegWrite=1;
                end
        endcase
    end



endmodule

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

module Adder32Bit(input1, input2, out);
    input [31:0] input1, input2;
    output [31:0] out;
    reg [31:0]out;
    always@( input1 or input2)
        begin
            out <= input1 + input2;
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
module Register_File (clk,read_addr_1, read_addr_2, write_addr, read_data_1, read_data_2, write_data, RegWrite);
	input [4:0] read_addr_1, read_addr_2, write_addr;
	input [31:0] write_data;
	input  clk,RegWrite;
	reg checkRegWrite;
	output reg [31:0] read_data_1, read_data_2;
	reg [31:0] Regfile [31:0];
	integer k;
	initial begin
	for (k=0; k<32; k=k+1) 
			begin
				Regfile[k] = 32'd10;
			end
	Regfile[0]=10;
    Regfile[8]=22;
    Regfile[9]=21;
    Regfile[5]=77;
    Regfile[7]=99;
    Regfile[21]=55;
    Regfile[20]=55;
    
	end
	
	//assign read_data_1 = Regfile[read_addr_1];
        always @(read_data_1 or Regfile[read_addr_1])
	        begin
	          if (read_addr_1 == 0) read_data_1 = 0;
	          else 
	          begin
	          read_data_1 = Regfile[read_addr_1];
	          //$display("read_addr_1=%d,read_data_1=%h",read_addr_1,read_data_1);
	          end
	        end
	//assign read_data_2 = Regfile[read_addr_2];
        always @(read_data_2 or Regfile[read_addr_2])
	        begin
	          if (read_addr_2 == 0) read_data_2 = 0;
	          else 
	          begin
	          read_data_2 = Regfile[read_addr_2];
	          //$display("read_addr_2=%d,read_data_2=%h",read_addr_2,read_data_2);
	          end
	        end
	always @(posedge clk)
	        begin
		      if (RegWrite == 1'b1)
		         begin 
		             Regfile[write_addr] = write_data;
		             $display("write_addr=%d write_data=%h",write_addr,write_data);
		         end
	        end
endmodule

module Data_Memory (addr, write_data, read_data, MemRead, MemWrite);
	input [31:0] addr;
	input [31:0] write_data;
	output [31:0] read_data;
	input MemRead, MemWrite;
	reg [31:0] DMemory [63:0];
	integer k;
	assign read_data = (MemRead) ? DMemory[addr] : 32'bx;
//	initial begin
		//for (k=0; k<64; k=k+1) 
            //begin
		      //     DMemory[k] = 32'b0;
			//end
		//DMemory[11] = 99;
	//end
	always @(*)
	begin
		if (MemWrite) DMemory[addr] = write_data;
	end
endmodule

module Sign_Extension (sign_in, sign_out);
	input [15:0] sign_in;
	output [31:0] sign_out;
	assign sign_out[15:0]=sign_in[15:0];
	assign sign_out[31:16]=sign_in[15]?16'b1111_1111_1111_1111:16'b0;
endmodule

module Mux_32_bit (in0, in1, mux_out, select);
	parameter N = 32;
	input [N-1:0] in0, in1;
	output [N-1:0] mux_out;
	input select;
	assign mux_out = select? in1: in0 ;
endmodule

module Mux_5_bit (in0, in1, mux_out, select);
	parameter N = 5;
	input [N-1:0] in0, in1;
	output [N-1:0] mux_out;
	input select;
	assign mux_out = select? in1: in0 ;
endmodule

module Instruction_Memory (read_address, instruction, reset);
	input reset;
	input [31:0] read_address;
	output [31:0] instruction;
	reg [31:0] Imemory [63:0];
	integer k;
	// I-MEM in this case is addressed by word, not by byte
	assign instruction = Imemory[read_address];
	always @(posedge reset)
	begin
	for (k=16; k<32; k=k+1) 
		begin  
		  // here Out changes k=0 to k=16
		  Imemory[k] = 32'b0;
		end
    //addi $s2, $zero, 0x55 //  load immediate value 0x55 to register $s2
    Imemory[0] = 32'b00100000000100100000000001010101;
    //addi $s3, $zero, 0x22 //  load immediate value 0x22 to register $s3
    Imemory[1] = 32'b00100000000100110000000000100010;
    //addi $s5, $zero, 0x77 //  load immediate value 0x77 to register $s5
    Imemory[2] = 32'b00100000000101010000000001110111;
    //add $s4,  $s2, $s3	    //	$s4 = $s2 + $s3  => R20=0x77 
    Imemory[3] = 32'b00000010010100111010000000100000;
    //sub $s1, $s2, $s3	    //	$s1 = $s2 – $s3  => R17=0x22 
    Imemory[4] = 32'b00000010010100111000100000100010;
    //sw  $s1, 0x02($s2)	    //	Memory[$s2+0x02] = $s1
    Imemory[5] = 32'b10101110010100010000000000000010;
    //lw $s6, 0x02($s2)	        //	$s6 = Memory[$s2+0x02] 
    Imemory[6] = 32'b10001110010101100000000000000010;
    //beq $s4,$s5, End      //beq $s4,$s5, 0x03
    Imemory[7] = 32'b00010010100101010000000000000011;
    //addi $s7, $zero, 0x10
    Imemory[8] = 32'b00100000000101110000000000010000;
    //bne $s5, $s4, End	    //bne $s4, $s5, 0x01
    Imemory[9] = 32'b00010110100101010000000000000001;
    //addi $s7, $zero, 0x20 
    Imemory[10] = 32'b00100000000101110000000000100000;
    //j 0x0b
    Imemory[11] = 32'b10001000000000000000000000000000;


	end
endmodule

module Sign_Extension_26 (sign_in, sign_out);
	input [25:0] sign_in;
	output [31:0] sign_out;
	assign sign_out[25:0]=sign_in[25:0];
	assign sign_out[31:26]=sign_in[25]?6'b111111:6'b0;
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