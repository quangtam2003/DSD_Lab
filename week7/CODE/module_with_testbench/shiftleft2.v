module shift_left_2 (sign_in, sign_out);
    input [31:0] sign_in;
    output [31:0] sign_out;
    assign sign_out[31:2]=sign_in[29:0];
    assign sign_out[1:0]=2'b00;
endmodule

module testbench;
    reg [31:0]sign_in;
    wire [31:0]sign_out;
    shift_left_2 tb(sign_in, sign_out);
    initial begin
        $monitor("in=%b out=%b",sign_in, sign_out);
        sign_in = 32'b11100000000000000000000000001111;
    end
endmodule