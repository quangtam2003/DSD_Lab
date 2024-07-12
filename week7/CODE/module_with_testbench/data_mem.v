module testbench;
    reg [7:0] addr;
    reg [31:0] write_data;
    wire [31:0] read_data;
    reg MemRead, MemWrite,clk;

    Data_Memory dut (clk,addr, write_data, read_data, MemRead, MemWrite);

    always #5 clk=~clk;

    initial begin
        $monitor($time, " clk=%b, addr=%b, write_data=%h, read_data=%h, MemRead=%b, MemWrite=%b",clk,addr, write_data, read_data, MemRead, MemWrite);
        clk=0; MemRead=1'b1; MemWrite=1'b0; addr=7'b100;
        #11 addr=7'b1000;
        #1 MemRead=1'b0;
        #5 MemWrite=1'b1; addr=7'b100; write_data=8'haa;
        #9 MemRead=1'b1; MemWrite=1'b0;
        #4 $finish;

    end
endmodule


module Data_Memory (clk,addr, write_data, read_data, MemRead, MemWrite);
    input [7:0] addr;
    input [31:0] write_data;
    output [31:0] read_data;
    input MemRead, MemWrite,clk;
    reg [31:0] DMemory [255:0];
    integer k;
    initial begin
        for (k=0; k<256; k=k+1)
            begin
                DMemory[k] = 32'b0;
            end
        //sw  $s1, 0x02($s2)        //  Memory[$s2+0x02] = $s1
        DMemory[0] = 32'b10101110010100010000000000000010;       
        
        //add $s4,  $s2, $s3        //  $s4 = $s2 + $s3  => R20=0x90 
        DMemory[4] = 32'b00000010010100111010000000100000;
        
        
        //add $s5 $t0 $t1        
        DMemory[8] = 32'b00000001000010011010100000100000;
        
        
        //sub $s1, $s2, $s3     //  $s1 = $s2 – $s3  => R17=0x22 
        DMemory[12] = 32'b00000010010100111000100000100010;
        
        //sw  $s1, 0x02($s2)        //  Memory[$s2+0x02] = $s1
        DMemory[16] = 32'b10101110010100010000000000000010;
        
        
        //lw $s1, 0x02($s2)         //  $s1 = Memory[$s2+0x02] 
        DMemory[20] = 32'b10001110010100010000000000000010;
        
        
        //beq $t2,$t3, End      //beq $t2,$t3, 0x03
        DMemory[24] = 32'b00010001010010110000000000000011;
        
        //addi $s7, $zero, 0x10
        DMemory[28] = 32'b00100000000101110000000000010000;
                //j 0x00
        DMemory[32] = 32'b00001000000000000000000000000000;
        //addi $s2, $zero, 0x55 //  load immediate value 0x55 to register $s2
        DMemory[36] = 32'b00100000000100100000000001010101;
        //addi $s3, $zero, 0x22 //  load immediate value 0x22 to register $s3
        DMemory[40] = 32'b00100000000100110000000000100010;
        //addi $s5, $zero, 0x77 //  load immediate value 0x77 to register $s5
        DMemory[44] = 32'b00100000000101010000000001110111;
        end
        
    assign read_data = (MemRead) ? DMemory[addr] : 32'bx;
    
    always @(posedge clk)
        begin
            if (MemWrite)
            begin
               DMemory[addr] = write_data;
               $display("Data memory write_addr=%b write_data=%h",addr,write_data);
            end
        end
endmodule