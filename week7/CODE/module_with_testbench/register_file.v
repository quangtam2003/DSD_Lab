module testbench;
    reg     [4:0] read_addr_1, read_addr_2, write_addr;
    reg     [31:0] write_data;
    reg     clk,RegWrite;
    wire    [31:0] read_data_1, read_data_2;

    Register_File DUT (clk,read_addr_1, read_addr_2, write_addr, read_data_1, read_data_2, write_data, RegWrite);
    always #5 clk=~clk;
    initial begin
        clk=0;
        $monitor($time, " clk=%b,read_addr_1=%d, read_addr_2=%d,  read_data_1=%d, read_data_2=%d, RegWrite=%d, write_addr=%d, write_data=%d, ", clk, read_addr_1, read_addr_2, read_data_1, read_data_2, RegWrite, write_addr, write_data);
            RegWrite=0; write_addr=32'd10; write_data=32'd10;
        #10 RegWrite=1;
        #25 write_addr=32'b1011; write_data=32'b1011;
        #5  RegWrite=0; read_addr_1=32'd10;
        #5  read_addr_2=32'b1011;
        #10 $finish;
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
