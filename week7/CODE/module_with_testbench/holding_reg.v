
module testbench;
  reg [31:0]input_data;
  reg write,clk,reset;
  wire [31:0] output_data;
  holding_reg DUT(output_data, input_data, write, clk, reset);
  always #5 clk=~clk;
    initial
    begin
      $monitor($time," output_data= %d input_data=%d write = %b clk=%d reset =%d",output_data, input_data, write, clk, reset);
      reset = 1; clk= 0;
      #10 reset = 0;write = 1;input_data = 16;
      #10 input_data = input_data +1;
      #10 write = 0;input_data = input_data +1;

      #10 $finish;
    end
endmodule

module holding_reg(output_data, input_data, write, clk, reset);
  // data size
  parameter word_size = 32;
  // inputs
  input [word_size-1:0] input_data;
  input write, clk, reset;

  // outputs
  output [word_size-1:0] output_data;

  // Register content and output assignment
  reg [word_size-1:0] content;
  assign output_data = content;

  // update regisiter contents
  always @(posedge clk) begin
    if (reset) begin
      content <= 0;
    end
    else if (write) begin
      content <= input_data;
    end
  end
endmodule
