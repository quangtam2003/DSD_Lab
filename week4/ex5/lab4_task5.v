module lab4_task5(SW,LEDR,LEDG,KEY,HEX7, HEX6, HEX5, HEX4, HEX3, HEX2, HEX1, HEX0);
	input [17:0] SW;
    input [3:0] KEY;
    output [0:6] HEX7, HEX6, HEX5, HEX4, HEX3, HEX2, HEX1, HEX0;
	output [17:0] LEDR;
	output [7:0] LEDG;
	wire [15:0]  W_LED_SEG;
	
	wire [7:0]   W_READ_ADDRESS;
	
	hex_ssd (W_LED_SEG[3:0], HEX0);
	hex_ssd (W_LED_SEG[7:4], HEX1);
	hex_ssd (W_LED_SEG[11:8],HEX2);
	hex_ssd (W_LED_SEG[15:12],HEX3);	
	
	hex_ssd (W_READ_ADDRESS[3:0],HEX6);
	hex_ssd (W_READ_ADDRESS[4:4],HEX7);
	
	data_path_instruction_mem(.reset(SW[17]),.clk(KEY[0]),.read_address(W_READ_ADDRESS),.instruction(W_LED_SEG));
endmodule

module hex_ssd (BIN, SSD);
  input [3:0] BIN;
  output reg [0:6] SSD;

  always begin
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

module data_path_instruction_mem(reset,clk,read_address,instruction);
input reset,clk;
output [7:0] read_address;
output [15:0] instruction;
wire [7:0] W_PC_add_1;

Program_Counter    C1(.clk(clk), .reset(reset), .PC_in(W_PC_add_1), .PC_out(read_address));
Adder8Bit          C2(.input1(8'b1), .input2(read_address), .out(W_PC_add_1));
Instruction_Memory C3(.reset(reset),.read_address(read_address), .instruction(instruction));
endmodule

module Instruction_Memory (reset,read_address, instruction);
	input reset;
	input [7:0] read_address;
	output [15:0] instruction;
	reg [15:0] Imemory [255:0];
	integer k;
	// I-MEM in this case is addressed by word, not by byte
	assign instruction = Imemory[read_address];

	always @(posedge reset)
	begin
	for (k=0; k<39; k=k+1) 
begin  
// here Out changes k=0 to k=16
			      Imemory[k] = 16'b0;
			end
Imemory[0] = 32'b00100000000010000000000000100000; 
//addi $t0, $zero, 32
Imemory[1] = 32'b00100000000010010000000000110111; 
//addi $t1, $zero, 55
Imemory[2] = 32'b00000001000010011000000000100100; 
//and $s0, $t0, $t1
Imemory[3] = 32'b00000001000010011000000000100101; 
//or $s0, $t0, $t1
Imemory[4] = 32'b10101100000100000000000000000100; 
//sw $s0, 4($zero)
Imemory[5] = 32'b10101100000010000000000000001000; 
//sw $t0, 8($zero)
Imemory[6] = 32'b00000001000010011000100000100000; 
//add $s1, $t0, $t1
Imemory[7] = 32'b00000001000010011001000000100010; 
//sub $s2, $t0, $t1
Imemory[8] = 32'b00010010001100100000000000001001; 
//beq $s1, $s2, error0
Imemory[9] = 32'b10001100000100010000000000000100; 
//lw $s1, 4($zero)
Imemory[10]= 32'b00110010001100100000000001001000; 
//andi $s2, $s1, 48
Imemory[11] =32'b00010010001100100000000000001001; 
//beq $s1, $s2, error1
Imemory[12] =32'b10001100000100110000000000001000; 
//lw $s3, 8($zero)
Imemory[13] =32'b00010010000100110000000000001010; 
//beq $s0, $s3, error2
Imemory[14] =32'b00000010010100011010000000101010; 
//slt $s4, $s2, $s1 (Last)
Imemory[15] =32'b00010010100000000000000000001111; 
//beq $s4, $0, EXIT
Imemory[16] =32'b00000010001000001001000000100000; 
//add $s2, $s1, $0
Imemory[17] =32'b00001000000000000000000000001110; 
//j Last
Imemory[18] =32'b00100000000010000000000000000000; 
//addi $t0, $0, 0(error0)
Imemory[19] =32'b00100000000010010000000000000000; 
//addi $t1, $0, 0
Imemory[20] =32'b00001000000000000000000000011111; 
//j EXIT
Imemory[21] =32'b00100000000010000000000000000001; 
//addi $t0, $0, 1(error1)
Imemory[22] =32'b00100000000010010000000000000001; 
//addi $t1, $0, 1
Imemory[23] =32'b00001000000000000000000000011111; 
//j EXIT
Imemory[24] =32'b00100000000010000000000000000010; 
//addi $t0, $0, 2(error2)
Imemory[25] =32'b00100000000010010000000000000010; 
//addi $t1, $0, 2
Imemory[26] =32'b00001000000000000000000000011111; 
//j EXIT
Imemory[27] =32'b00100000000010000000000000000011; 
//addi $t0, $0, 3(error3)
Imemory[28] =32'b00100000000010010000000000000011; 
//addi $t1, $0, 3
Imemory[29] =32'b00001000000000000000000000011111; 
//j EXIT 
end
endmodule

module Program_Counter (clk, reset, PC_in, PC_out);
	input clk, reset;
	input [7:0] PC_in;
	output [7:0] PC_out;
	reg [7:0] PC_out;
	always @ (posedge clk or posedge reset)
	begin
      if(reset==1'b1)
			PC_out<=8'b0;
		else
			PC_out<=PC_in;
	end
endmodule

module Adder8Bit(input1, input2, out);

 input [7:0] input1, input2;
 output [7:0] out;
 reg [7:0]out;
 always@( input1 or input2)
 begin
 	out  <= input1 + input2;
 end
endmodule
