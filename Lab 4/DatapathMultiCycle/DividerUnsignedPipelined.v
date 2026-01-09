`timescale 1ns / 1ns

// quotient = dividend / divisor

module DividerUnsignedPipelined (
    input             clk, rst, stall,
    input      [31:0] i_dividend,
    input      [31:0] i_divisor,
    output     [31:0] o_remainder,
    output     [31:0] o_quotient
);

  reg [31:0] dividend_reg [8:0];
  reg [31:0] divisor_reg  [8:0];
  reg [31:0] remainder_reg[8:0];
  reg [31:0] quotient_reg [8:0];

  wire [31:0] dividend_out [7:0];
  wire [31:0] remainder_out[7:0];
  wire [31:0] quotient_out [7:0];

  genvar i;
  generate
    for (i = 0; i < 8; i = i + 1) begin : stage
      wire [31:0] dividend_iter[4:0];
      wire [31:0] remainder_iter[4:0];
      wire [31:0] quotient_iter[4:0];

      assign dividend_iter[0]  = dividend_reg[i];
      assign remainder_iter[0] = remainder_reg[i];
      assign quotient_iter[0]  = quotient_reg[i];

      divu_1iter iter1 (
          .i_dividend(dividend_iter[0]), .i_divisor(divisor_reg[i]), .i_remainder(remainder_iter[0]), .i_quotient(quotient_iter[0]),
          .o_dividend(dividend_iter[1]), .o_remainder(remainder_iter[1]), .o_quotient(quotient_iter[1])
      );

      divu_1iter iter2 (
          .i_dividend(dividend_iter[1]), .i_divisor(divisor_reg[i]), .i_remainder(remainder_iter[1]), .i_quotient(quotient_iter[1]),
          .o_dividend(dividend_iter[2]), .o_remainder(remainder_iter[2]), .o_quotient(quotient_iter[2])
      );

      divu_1iter iter3 (
          .i_dividend(dividend_iter[2]), .i_divisor(divisor_reg[i]), .i_remainder(remainder_iter[2]), .i_quotient(quotient_iter[2]),
          .o_dividend(dividend_iter[3]), .o_remainder(remainder_iter[3]), .o_quotient(quotient_iter[3])
      );

      divu_1iter iter4 (
          .i_dividend(dividend_iter[3]), .i_divisor(divisor_reg[i]), .i_remainder(remainder_iter[3]), .i_quotient(quotient_iter[3]),
          .o_dividend(dividend_iter[4]), .o_remainder(remainder_iter[4]), .o_quotient(quotient_iter[4])
      );

      assign dividend_out[i]  = dividend_iter[4];
      assign remainder_out[i] = remainder_iter[4];
      assign quotient_out[i]  = quotient_iter[4];
    end
  endgenerate

  always @(posedge clk) begin : pipeline_logic
    integer j;
    if (rst) begin
      for (j = 0; j < 9; j = j + 1) begin
        dividend_reg[j]  <= 32'b0;
        divisor_reg[j]   <= 32'b0;
        remainder_reg[j] <= 32'b0;
        quotient_reg[j]  <= 32'b0;
      end
    end else if (!stall) begin
      dividend_reg[0]  <= i_dividend;
      divisor_reg[0]   <= i_divisor;
      remainder_reg[0] <= 32'b0;
      quotient_reg[0]  <= 32'b0; 

      for (j = 0; j < 8; j = j + 1) begin
        dividend_reg[j+1]  <= dividend_out[j];
        divisor_reg[j+1]   <= divisor_reg[j];
        remainder_reg[j+1] <= remainder_out[j];
        quotient_reg[j+1]  <= quotient_out[j];
      end
    end
  end
  
  assign o_remainder = remainder_reg[8];
  assign o_quotient  = quotient_reg[8];

endmodule

module divu_1iter (
    input      [31:0] i_dividend,
    input      [31:0] i_divisor,
    input      [31:0] i_remainder,
    input      [31:0] i_quotient,
    output     [31:0] o_dividend,
    output     [31:0] o_remainder,
    output     [31:0] o_quotient
);

    wire [32:0] remainder_shifted;
    assign remainder_shifted = {i_remainder, i_dividend[31]};

    wire is_greater_or_equal;
    assign is_greater_or_equal = (remainder_shifted >= i_divisor);

    assign o_quotient = (i_quotient << 1) | (is_greater_or_equal ? 32'd1 : 32'd0);

    assign o_remainder = is_greater_or_equal ? (remainder_shifted - i_divisor) : remainder_shifted;

    assign o_dividend = i_dividend << 1;

endmodule