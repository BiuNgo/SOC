module SystemDemo(
   input wire [3:0] btn,
   output wire [5:0] led
);
   wire [31:0] sum;
   cla cla_inst(
     .a(32'd26),
     .b({28'd0, btn}),
     .cin(1'b0),
     .sum(sum)
   );
   
   assign led = (sum > 32'd31) ? 6'b100000 : {1'b0, sum[4:0]};
endmodule