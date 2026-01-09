`timescale 1ns / 1ps

/**
 * @param a first 1-bit input
 * @param b second 1-bit input
 * @param g whether a and b generate a carry
 * @param p whether a and b would propagate an incoming carry
 */
module gp1(input wire a, b,
           output wire g, p);
   assign g = a & b;
   assign p = a | b;
endmodule

/**
 * Computes aggregate generate/propagate signals over a 4-bit window.
 * @param gin incoming generate signals
 * @param pin incoming propagate signals
 * @param cin the incoming carry
 * @param gout whether these 4 bits internally would generate a carry-out (independent of cin)
 * @param pout whether these 4 bits internally would propagate an incoming carry from cin
 * @param cout the carry outs for the low-order 3 bits
 */
module gp4(input wire [3:0] gin, pin,
           input wire cin,
           output wire gout, pout,
           output wire [2:0] cout);

   // TODO: your code here
   assign pout = &pin;
   assign gout = gin[3] | (pin[3] & gin[2]) | (pin[3] & pin[2] & gin[1]) | (pin[3] & pin[2] & pin[1] & gin[0]);

   assign cout[0] = gin[0] | (pin[0] & cin);
   assign cout[1] = gin[1] | (pin[1] & gin[0]) | (pin[1] & pin[0] & cin);
   assign cout[2] = gin[2] | (pin[2] & gin[1]) | (pin[2] & pin[1] & gin[0]) | (pin[2] & pin[1] & pin[0] & cin);

endmodule

/** Same as gp4 but for an 8-bit window instead */
module gp8(input wire [7:0] gin, pin,
           input wire cin,
           output wire gout, pout,
           output wire [6:0] cout);

   // TODO: your code here
   wire g_low, p_low;
   wire g_high, p_high;
   wire c4;
   
   gp4 low_block (
       .gin(gin[3:0]), 
       .pin(pin[3:0]), 
       .cin(cin), 
       .gout(g_low), 
       .pout(p_low), 
       .cout(cout[2:0])
   );
   
   assign c4 = g_low | (p_low & cin);
   assign cout[3] = c4;
   
   gp4 high_block (
       .gin(gin[7:4]), 
       .pin(pin[7:4]), 
       .cin(c4), 
       .gout(g_high), 
       .pout(p_high), 
       .cout(cout[6:4])
   );
   
   assign pout = p_high & p_low;
   assign gout = g_high | (p_high & g_low);

endmodule

module cla
  (input wire [31:0]  a, b,
   input wire         cin,
   output wire [31:0] sum);

   // TODO: your code here
   wire [31:0] g, p;
   wire [3:0] G_block, P_block;
   wire [2:0] C_block; 
   wire [6:0] c_internal_0, c_internal_1, c_internal_2, c_internal_3;
   
   genvar i;
   generate
      for (i=0; i<32; i=i+1) begin : gp1_inst
         gp1 bit_gp (.a(a[i]), .b(b[i]), .g(g[i]), .p(p[i]));
      end
   endgenerate
   
   gp8 block0 (
       .gin(g[7:0]), 
       .pin(p[7:0]), 
       .cin(cin), 
       .gout(G_block[0]), 
       .pout(P_block[0]), 
       .cout(c_internal_0)
   );
   
   wire G_unused, P_unused;
   
   gp4 cla_tree_root (
       .gin(G_block),
       .pin(P_block), 
       .cin(cin), 
       .gout(G_unused), 
       .pout(P_unused), 
       .cout(C_block)
   );
   
   gp8 block1 (
       .gin(g[15:8]), 
       .pin(p[15:8]), 
       .cin(C_block[0]),
       .gout(G_block[1]), 
       .pout(P_block[1]), 
       .cout(c_internal_1)
   );
   
   gp8 block2 (
       .gin(g[23:16]), 
       .pin(p[23:16]), 
       .cin(C_block[1]),
       .gout(G_block[2]), 
       .pout(P_block[2]), 
       .cout(c_internal_2)
   );
   
   gp8 block3 (
       .gin(g[31:24]), 
       .pin(p[31:24]), 
       .cin(C_block[2]),
       .gout(G_block[3]), 
       .pout(P_block[3]), 
       .cout(c_internal_3)
   );
   
   wire [31:0] full_carries;
   
   assign full_carries[0]     = cin;
   assign full_carries[7:1]   = c_internal_0;
   
   assign full_carries[8]     = C_block[0];
   assign full_carries[15:9]  = c_internal_1;
   
   assign full_carries[16]    = C_block[1];
   assign full_carries[23:17] = c_internal_2;
   
   assign full_carries[24]    = C_block[2];
   assign full_carries[31:25] = c_internal_3;

   assign sum = a ^ b ^ full_carries;

endmodule
