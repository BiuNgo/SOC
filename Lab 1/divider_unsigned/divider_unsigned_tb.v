`timescale 1ns / 1ps

module divider_unsigned_tb;

    reg  [31:0] i_dividend;
    reg  [31:0] i_divisor;
    
    wire [31:0] o_remainder;
    wire [31:0] o_quotient;

    integer i;

    divider_unsigned uut (
        .i_dividend (i_dividend),
        .i_divisor  (i_divisor),
        .o_remainder(o_remainder),
        .o_quotient (o_quotient)
    );

    initial begin
        
        i_dividend = 0;
        i_divisor = 1;
        #1;

        i_dividend = 4;
        i_divisor  = 2;
        #1;

        i_dividend = 4;
        i_divisor  = 4;
        #1;

        i_dividend = 10;
        i_divisor  = 4;
        #1;

        i_dividend = 2;
        i_divisor  = 4;
        #1;

        for (i = 0; i < 1000; i = i + 1) begin
            
            i_dividend = $urandom; 
            i_divisor  = $urandom;
            
            if (i_divisor == 0) i_divisor = 1;

            #1;

        end

        $finish;
    end

endmodule