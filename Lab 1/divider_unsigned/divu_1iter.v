module divu_1iter (
    input  wire [31:0] i_dividend,
    input  wire [31:0] i_divisor,
    input  wire [31:0] i_remainder,
    input  wire [31:0] i_quotient,
    output wire [31:0] o_dividend,
    output wire [31:0] o_remainder,
    output wire [31:0] o_quotient
);

    wire [31:0] remainder_shifted;
    assign remainder_shifted = {i_remainder[30:0], i_dividend[31]};

    wire is_greater_or_equal;
    assign is_greater_or_equal = (remainder_shifted >= i_divisor);

    assign o_quotient = (i_quotient << 1) | (is_greater_or_equal ? 32'd1 : 32'd0);

    assign o_remainder = is_greater_or_equal ? (remainder_shifted - i_divisor) : remainder_shifted;

    assign o_dividend = i_dividend << 1;

endmodule