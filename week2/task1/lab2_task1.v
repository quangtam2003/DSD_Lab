module lab2_task1(SW,LEDR,LEDG);
	input [17:0] SW;
	output [7:0] LEDR;
	output [17:0] LEDG;
	assign LEDR=SW;
	D_flipflop DUT(.clk(SW[2]), .rst_n(SW[1]),.d(SW[0]),.q(LEDG[0]));
endmodule

module D_flipflop (
  input clk, rst_n,
  input d,
  output reg q
  );
  always@(posedge clk or negedge rst_n) begin
    if(!rst_n) q <= 0;
    else       q <= d;
  end
endmodule