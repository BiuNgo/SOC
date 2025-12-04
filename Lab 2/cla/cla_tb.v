`timescale 1ns / 1ps

module cla_tb;

   reg [31:0] a;
   reg [31:0] b;
   reg cin;

   wire [31:0] sum;

   cla uut (
       .a(a), 
       .b(b), 
       .cin(cin), 
       .sum(sum)
   );

   integer i;

   initial begin
      a = 0; b = 0; cin = 0;
      #10;

      a = 0; b = 1; cin = 0;
      #10;

      a = 1; b = 0; cin = 0;
      #10;

      a = 1; b = 1; cin = 0;
      #10;

      a = 1; b = 1; cin = 1;
      #10;

      a = 32'hFFFFFFFF; b = 0; cin = 1;
      #10;

      a = 32'hAAAAAAAA; b = 32'h55555555; cin = 1;
      #10;

      for (i = 0; i < 1000; i = i + 1) begin
         a = $random;
         b = $random;
         cin = $random & 1;

         #10;
      end
   end

endmodule