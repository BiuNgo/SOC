`timescale 1ns / 1ps

module tb_gp4;

   // Inputs
   reg [3:0] gin;
   reg [3:0] pin;
   reg cin;

   // Outputs
   wire gout;
   wire pout;
   wire [2:0] cout;

   // Instantiate the Unit Under Test (UUT)
   gp4 uut (
       .gin(gin), 
       .pin(pin), 
       .cin(cin), 
       .gout(gout), 
       .pout(pout), 
       .cout(cout)
   );

   initial begin
      // Optional: View waveforms
      $dumpfile("tb_gp4.vcd");
      $dumpvars(0, tb_gp4);
      
      // Print results to console automatically when signals change
      $monitor("Time=%0t | gin=%h pin=%h cin=%b | gout=%b pout=%b cout=%b", 
               $time, gin, pin, cin, gout, pout, cout);

      // 1. Zeroes
      gin = 4'h0; pin = 4'h0; cin = 0;
      #10;

      // 2. MSB Generate (gin[3] is high)
      gin = 4'h8; pin = 4'h0; cin = 0;
      #10;

      // 3. Propagate Full (cin ripples through)
      gin = 4'h0; pin = 4'hF; cin = 1;
      #10;

      // 4. Propagate Partway (pin[3] is 0)
      gin = 4'h0; pin = 4'h7; cin = 1;
      #10;

      // 5. Propagate Full No Carry
      gin = 4'h0; pin = 4'hF; cin = 0;
      #10;

      // 6. Propagate and Generate
      gin = 4'hF; pin = 4'hF; cin = 1;
      #10;

      $finish;
   end

endmodule