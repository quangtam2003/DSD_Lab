module testbench;
  reg clk;
  reg [3:0] PC_in;
  reg [27:0] IR_in;
  wire[31:0] c_out;

  concate DUT (PC_in,IR_in,c_out);

  initial begin
    $monitor("PC_in=%h IR_in=%h PC_out=%h",PC_in,IR_in,c_out);
    clk=0;
    PC_in=4'ha;
    IR_in=28'hface666;
    #5
    PC_in=4'hb;
    IR_in=28'hface666;    
    
    #5 $finish;
  end
endmodule

module concate(PC_in,IR_in,PC_out);
    input [3:0] PC_in;
    input [27:0] IR_in;
    output[31:0] PC_out;
    assign PC_out={PC_in, IR_in};
endmodule

