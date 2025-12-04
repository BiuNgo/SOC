module divider_unsigned (
    input  wire [31:0] i_dividend,
    input  wire [31:0] i_divisor,
    output wire [31:0] o_remainder,
    output wire [31:0] o_quotient
);

    wire [31:0] dividend_chain  [32:0];
    wire [31:0] remainder_chain [32:0];
    wire [31:0] quotient_chain  [32:0];

    assign dividend_chain[0]  = i_dividend;
    assign remainder_chain[0] = 32'd0;
    assign quotient_chain[0]  = 32'd0;

    genvar i;
    generate
        for (i = 0; i < 32; i = i + 1) begin : gen_div_steps
            divu_1iter u_iter (
                .i_dividend  (dividend_chain[i]),
                .i_divisor   (i_divisor),
                .i_remainder (remainder_chain[i]),
                .i_quotient  (quotient_chain[i]),
                .o_dividend  (dividend_chain[i+1]),
                .o_remainder (remainder_chain[i+1]),
                .o_quotient  (quotient_chain[i+1])
            );
        end
    endgenerate

    assign o_remainder = remainder_chain[32];
    assign o_quotient  = quotient_chain[32];

endmodule