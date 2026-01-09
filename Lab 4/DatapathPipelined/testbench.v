`timescale 1ns / 1ns

module testbench;

  reg clk;
  reg rst;
  reg stall;
  reg [31:0] i_dividend;
  reg [31:0] i_divisor;

  wire [31:0] o_remainder;
  wire [31:0] o_quotient;

  integer i;
  integer error_count = 0;
  
  localparam TEST_COUNT = 160;
  localparam LATENCY    = 9;

  reg [31:0] save_dividend  [0:TEST_COUNT-1];
  reg [31:0] save_divisor   [0:TEST_COUNT-1];
  reg [31:0] exp_quotient   [0:TEST_COUNT-1];
  reg [31:0] exp_remainder  [0:TEST_COUNT-1];

  DividerUnsignedPipelined uut (
    .clk(clk),
    .rst(rst),
    .stall(stall),
    .i_dividend(i_dividend),
    .i_divisor(i_divisor),
    .o_remainder(o_remainder),
    .o_quotient(o_quotient)
  );

  initial begin
    clk = 0;
    forever #1 clk = ~clk;
  end

  initial begin
    $display("------------------------------------------------");
    $display("Starting test0: 4 / 2");
    rst = 1;
    stall = 0;
    i_dividend = 0;
    i_divisor = 0;
    @(posedge clk);
    @(posedge clk);
    rst = 0;
    @(posedge clk);

    i_dividend = 4;
    i_divisor = 2;
    repeat (LATENCY) @(posedge clk);

    #0.1; 
    if (o_quotient === 2 && o_remainder === 0) 
      $display("test0 PASSED");
    else 
      $error("test0 FAILED: Expected Q=2, R=0. Got Q=%d, R=%d", o_quotient, o_remainder);

    $display("Starting test1: 12 / 3");
    rst = 1; @(posedge clk); rst = 0; @(posedge clk);

    i_dividend = 12;
    i_divisor = 3;
    repeat (LATENCY) @(posedge clk);

    #0.1;
    if (o_quotient === 4 && o_remainder === 0) 
      $display("test1 PASSED");
    else 
      $error("test1 FAILED: Expected Q=4, R=0. Got Q=%d, R=%d", o_quotient, o_remainder);

    $display("Starting test2: Manual Consecutive");
    rst = 1; @(posedge clk); rst = 0; @(posedge clk);

    i_dividend = 4;  i_divisor = 2;
    @(posedge clk); 
    
    i_dividend = 12; i_divisor = 3;
    
    repeat (LATENCY - 1) @(posedge clk);
    
    #0.1;
    if (o_quotient === 2) $display("test2 Step A PASSED");
    else $error("test2 Step A FAILED");

    @(posedge clk); 
    #0.1;
    if (o_quotient === 4) $display("test2 Step B PASSED");
    else $error("test2 Step B FAILED");

    $display("------------------------------------------------");
    $display("Starting test3: Stress Test (%0d consecutive inputs)", TEST_COUNT);
    
    for (i = 0; i < TEST_COUNT; i = i + 1) begin
        save_dividend[i] = $random;
        save_divisor[i]  = $random; 
        
        if (save_divisor[i] == 0) save_divisor[i] = 1;
        
        exp_quotient[i]  = save_dividend[i] / save_divisor[i];
        exp_remainder[i] = save_dividend[i] % save_divisor[i];
    end

    rst = 1; @(posedge clk); rst = 0; @(posedge clk);

    for (i = 0; i < TEST_COUNT + LATENCY; i = i + 1) begin
        
        if (i < TEST_COUNT) begin
            i_dividend = save_dividend[i];
            i_divisor  = save_divisor[i];
        end else begin
            i_dividend = 0;
            i_divisor  = 1;
        end

        if (i >= LATENCY) begin
            #0.1; 
            if (o_quotient !== exp_quotient[i - LATENCY] || o_remainder !== exp_remainder[i - LATENCY]) begin
                $error("Stress Test FAILED at index %0d", (i - LATENCY));
                $display("  Input: %d / %d", save_dividend[i-LATENCY], save_divisor[i-LATENCY]);
                $display("  Exp: Q=%d, R=%d", exp_quotient[i-LATENCY], exp_remainder[i-LATENCY]);
                $display("  Got: Q=%d, R=%d", o_quotient, o_remainder);
                error_count = error_count + 1;
            end
        end

        @(posedge clk);
    end

    $display("------------------------------------------------");
    if (error_count == 0)
        $display("ALL TESTS PASSED SUCCESSFULLY!");
    else
        $display("COMPLETED WITH %0d ERRORS.", error_count);
        
    $finish;
  end

endmodule