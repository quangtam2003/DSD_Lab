module testbench;
    reg clk, reset;
    reg PCWrite,Iord,MemWrite,MemtoReg,RegWrite,RegDst,ALUSrcA,PCWr,IRwrite,MemRead;
    reg [2:0]ALUop;
    reg [1:0] ALUSrcB,PCSource;
    reg [2:0] Operation_ALU;
    
    wire [31:0] ALU_in_B,ALU_in_A,ALU_out,B_data,mux_2_out,Jump_addr,mux_1_out;
    wire [31:0] PC_in,PC_out,Mem_Read_data,instruction,MDR_out,ALU_out_hold;
    
    wire [27:0] jump_28_bit;

    Datapath_Multi_cycle_Processor dut(clk, reset,PCWrite,Iord,MemWrite,MemtoReg,RegWrite,RegDst,ALUSrcA,PCWr,IRwrite,MemRead,ALUop,ALUSrcB,PCSource,Operation_ALU,ALU_in_B,ALU_in_A,ALU_out,B_data,mux_2_out,Jump_addr,PC_in,PC_out,Mem_Read_data,instruction,MDR_out,ALU_out_hold,jump_28_bit,mux_1_out);

    always #5 clk=~clk;

    initial begin
        $monitor($time,,"clk=%b|reset=%b|PC_in=%h|PC_out=%h|mux_1_out=%h|Mem_Read_data=%b|instruction=%b|ALU_in_A=%h|ALU_in=%h|ALU_out=%h",clk,reset,PC_in[3:0],PC_out[3:0],mux_1_out[3:0],Mem_Read_data[31:26],instruction[31:26],ALU_in_A[3:0],ALU_in_B[3:0],ALU_out[3:0]);
            Iord=0;
            MemRead=1;
            MemWrite=0;
            IRwrite=1;
            MemtoReg=0;
            RegWrite=0;
            RegDst=0;
            ALUSrcA=0;
            ALUSrcB=2'b01;
            PCSource=2'b00;
            Operation_ALU=3'b000;
            PCWr=1;

                reset=1;clk=0;
            #15 reset=0;

            #15 $finish;
    end
endmodule


module  Datapath_Multi_cycle_Processor(clk, reset,PCWrite,Iord,MemWrite,MemtoReg,RegWrite,RegDst,ALUSrcA,PCWr,IRwrite,MemRead,ALUop,ALUSrcB,PCSource,Operation_ALU,ALU_in_B,ALU_in_A,ALU_out,B_data,mux_2_out,Jump_addr,PC_in,PC_out,Mem_Read_data,instruction,MDR_out,ALU_out_hold,jump_28_bit,mux_1_out);
    input clk, reset;
    input PCWrite,Iord,MemWrite,MemtoReg,RegWrite,RegDst,ALUSrcA,PCWr,IRwrite,MemRead;
    input [2:0]ALUop;
    input [1:0] ALUSrcB,PCSource;
    input [2:0] Operation_ALU;
    
    output [31:0] ALU_in_B,ALU_in_A,ALU_out,B_data,mux_2_out,Jump_addr,mux_1_out;
    output [31:0] PC_in,PC_out,Mem_Read_data,instruction,MDR_out,ALU_out_hold;
    
    output [27:0] jump_28_bit;
   
    wire [31:0] W_RD1, W_RD2,Extend_out,Branch_addr,A_data;
    wire [4:0] mux_3_out;
    wire zero,PCWrcond,and_out;

    Program_Counter     comp1(clk, reset,PCWr,PC_in, PC_out);
    Mux_32_bit          comp2(PC_out, ALU_out_hold, mux_1_out, Iord);
    Data_Memory         comp3(clk,mux_1_out, B_data, Mem_Read_data, MemRead, MemWrite);
    holding_reg         comp4(instruction, Mem_Read_data, IRwrite, clk, reset);
    holding_reg         comp5(MDR_out, Mem_Read_data, 1'b1, clk, reset);
    Mux_32_bit          comp6(MDR_out,ALU_out_hold, mux_2_out, MemtoReg);

    Register_File       comp7(clk,instruction[25:21], instruction[20:16], mux_3_out, W_RD1, W_RD2, mux_2_out, RegWrite);
    Mux_5_bit           comp8(instruction[20:16], instruction[15:11], mux_3_out, RegDst);
    Sign_Extension      comp9(instruction[15:0], Extend_out);
    shift_left_2        comp10(Extend_out, Branch_addr);
    holding_reg         comp11(A_data, W_RD1, 1'b1, clk, reset);
    holding_reg         comp12(B_data, W_RD2, 1'b1, clk, reset);
    Mux_32_bit          comp13(PC_out, A_data, ALU_in_A, ALUSrcA);
    Mux4_32_bit         comp14(B_data, 32'd4,Extend_out,Branch_addr , ALU_in_B, ALUSrcB);
    alu                 comp15(Operation_ALU, ALU_in_A, ALU_in_B, ALU_out,zero);
    holding_reg         comp16(ALU_out_hold, ALU_out , 1'b1, clk, reset);
    
    shift_left_2_28bit  comp17(instruction[25:0], jump_28_bit);
    
    concate             comp18(PC_out[31:28],jump_28_bit,Jump_addr);
    Mux4_32_bit         comp19(ALU_out, ALU_out_hold,Jump_addr, 32'b0, PC_in, PCSource);
    
endmodule

module Mux_5_bit (in0, in1, mux_out, select);
	parameter N = 5;
	input [N-1:0] in0, in1;
	output [N-1:0] mux_out;
	input select;
	assign mux_out = select? in1: in0 ;
endmodule

module Sign_Extension (sign_in, sign_out);
	input [15:0] sign_in;
	output [31:0] sign_out;
	assign sign_out[15:0]=sign_in[15:0];
	assign sign_out[31:16]=sign_in[15]?16'b1111_1111_1111_1111:16'b0;
endmodule

module Program_Counter (clk, reset,PC_write ,PC_in, PC_out);
	input clk, reset,PC_write;
	input [31:0] PC_in;
	output reg [31:0] PC_out;
	always @ (posedge clk or posedge reset)
	begin
		if(reset==1'b1)
			PC_out<=0;
		else if (PC_write==1'b1)
			PC_out<=PC_in;
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
			    ALU_OP_OR	    = 3'b011,
			    ALU_OP_XOR	    = 3'b100,
			    ALU_OP_LW	    = 3'b101,
			    ALU_OP_SW	    = 3'b110,
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
	initial 
	    begin
	        for (k=0; k<32; k=k+1) 
			    begin
				    Regfile[k] = 32'd10;
			    end
			Regfile[8]=32'd1;//$t0
			Regfile[9]=32'd2;//$t1
			Regfile[10]=32'd3; //$t2
			Regfile[11]=32'd4; //$t3
			
			
			Regfile[17]=32'd99;//$s1
			Regfile[18]=32'd60;//$s2
			Regfile[19]=32'd30;//$s3
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
		             $display("Rigister File write_addr=%d write_data=%d",write_addr,write_data);
		         end
	        end
endmodule

module holding_reg(output_data, input_data, write, clk, reset);
  // data size
  parameter N = 32;
  // inputs
  input [N-1:0] input_data;
  input	write, clk, reset;

  // outputs
  output [N-1:0] output_data;

  // Register content and output assignment
  reg [N-1:0] content;
    // update regisiter contents
  always @(posedge clk or write) 
  begin
    if (reset) 
    begin
      content <= 0;
    end
    else if (write) 
    begin
      content <= input_data;
    end
  end
  assign output_data = content;
endmodule

module Mux_32_bit (in0, in1, mux_out, select);
	parameter N = 32;
	input [N-1:0] in0, in1;
	output [N-1:0] mux_out;
	input select;
	assign mux_out = select? in1: in0 ;
endmodule

module shift_left_2 (sign_in, sign_out);
	input [31:0] sign_in;
	output [31:0] sign_out;
	assign sign_out[31:2]=sign_in[29:0];
	assign sign_out[1:0]=2'b00;
endmodule

module concate(PC_in,IR_in,PC_out);
    input [3:0] PC_in;
    input [27:0] IR_in;
    output[31:0] PC_out;
    assign PC_out={PC_in, IR_in};
endmodule

module Mux4_32_bit (in0, in1,in2, in3, mux_out, select);
	parameter N = 32;
	input [N-1:0] in0, in1,in2,in3;
	output [N-1:0] mux_out;
	input [1:0]select;
	assign mux_out = select[1]? (select[0]?in3: in2):(select[0]?in1:in0);
endmodule

module shift_left_2_28bit (sign_in, sign_out);
	input [25:0] sign_in;
	output [27:0] sign_out;
	assign sign_out={2'b00,sign_in};
endmodule

module Data_Memory (clk,addr, write_data, read_data, MemRead, MemWrite);
    input [31:0] addr;
    input [31:0] write_data;
    output [31:0] read_data;
    input MemRead, MemWrite,clk;
    reg [31:0] DMemory [63:0];
    integer k;
    initial begin
        for (k=0; k<64; k=k+1)
            begin
                DMemory[k] = 32'b0;
            end
        //sw  $s1, 0x02($s2)	    //	Memory[$s2+0x02] = $s1
        DMemory[0] = 32'b10101110010100010000000000000010;       
        
        //add $s4,  $s2, $s3	    //	$s4 = $s2 + $s3  => R20=0x90 
        DMemory[4] = 32'b00000010010100111010000000100000;
        
        
        //add $s5 $t0 $t1	    //r[21]=t0+t1=1+2=3 
        DMemory[8] = 32'b00000001000010011010100000100000;
        
        
        //sub $s1, $s2, $s3	    //	$s1 = $s2 – $s3  => R17=0x22=d30
        DMemory[12] = 32'b00000010010100111000100000100010;
        
        //sw  $s1, 0x02($s2)	    //	Memory[$s2+0x02] = $s1 = d30  //memory[62]=d30
        DMemory[16] = 32'b10101110010100010000000000000010;
        
        
        //lw $s1, 0x02($s2)	        //	$s1 = Memory[$s2+0x02]
        //R[17]=memory[62]=d30
        DMemory[20] = 32'b10001110010100010000000000000010;
        
        
        //beq $t2,$t3, End      //beq $t2,$t3, 0x03
        DMemory[24] = 32'b00010001010010110000000000000011;
        
        //addi $s7, $zero, 0x16  //R[23]=0x16
        DMemory[28] = 32'b00100000000101110000000000010000;
                
        //addi $s2, $zero, 0x55 //  load immediate value 0x55 to register $s2
        DMemory[32] = 32'b00100000000100100000000000110111;
        //addi $s3, $zero, 0x34 //  load immediate value 0x22 to register $s3
        DMemory[36] = 32'b00100000000100110000000000100010;
        //addi $s5, $zero, 0x119 //  load immediate value 0x77 to register $s5
        DMemory[40] = 32'b00100000000101010000000001110111;
        //j 0x00
        DMemory[44] = 32'b00001000000000000000000000000000;
        end
        
    assign read_data = (MemRead) ? DMemory[addr] : 32'bx;
    
    always @(posedge clk)
        begin
            if (MemWrite)
            begin
               DMemory[addr] = write_data;
               $display("Data memory write_addr=%d write_data=%d",addr,write_data);
            end
        end
endmodule
