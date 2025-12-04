`timescale 1ns / 1ps

module divu_1iter_tb;

    reg [31:0] i_dividend;
    reg [31:0] i_divisor;
    reg [31:0] i_remainder;
    reg [31:0] i_quotient;

    wire [31:0] o_dividend;
    wire [31:0] o_remainder;
    wire [31:0] o_quotient;
    
    integer k;

    divu_1iter uut (
        .i_dividend (i_dividend),
        .i_divisor  (i_divisor),
        .i_remainder(i_remainder),
        .i_quotient (i_quotient),
        .o_dividend (o_dividend),
        .o_remainder(o_remainder),
        .o_quotient (o_quotient)
    );

    initial begin
        i_dividend  = 0;
        i_divisor   = 1;
        i_remainder = 0;
        i_quotient  = 0;
        
        #1;
        i_dividend  = 32'h8000_0000;
        i_divisor   = 32'd2;
        i_remainder = 32'd0;
        i_quotient  = 32'd8;
        
        #1;
        i_dividend  = 32'h8000_0000;
        i_divisor   = 32'd1;
        i_remainder = 32'd0;
        i_quotient  = 32'd8;
        
        for (k = 0; k < 1000; k = k + 1) begin
            #1;
            i_dividend  = $urandom;
            i_divisor   = $urandom;
            i_remainder = $urandom;
            i_quotient  = $urandom;

            if (i_divisor == 0) i_divisor = 1;
        end

        $finish;
    end

endmodule