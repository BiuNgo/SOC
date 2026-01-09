`timescale 1ns / 1ns

module tb_Processor;

  // Inputs to DUT
  reg clk;
  reg rst;

  // Outputs from DUT
  wire halt;
  wire [31:0] trace_writeback_pc;
  wire [31:0] trace_writeback_inst;

  // Instantiate the Device Under Test (DUT)
  Processor uut (
      .clk(clk),
      .rst(rst),
      .halt(halt),
      .trace_writeback_pc(trace_writeback_pc),
      .trace_writeback_inst(trace_writeback_inst)
  );

  // Clock generation: 4ns period (250 MHz)
  initial begin
    clk = 0;
    forever #2 clk = ~clk;
  end

  // Test Logic
  initial begin
    // 1. Initialize Memory
    // We access the internal memory array of the DUT directly to hardcode the hex.
    // Initialize all to 0 first
    initialize_memory();

    // 2. Reset Sequence
    $display("Applying Reset...");
    rst = 1;
    #10; // Hold reset for a few cycles
    
    // 3. Start Execution
    @(negedge clk);
    rst = 0;
    $display("Reset released. Processor started.");

    // 4. Run Simulation
    // Wait for Halt or Timeout (1000 cycles)
    wait (halt || uut.datapath.cycles_current > 1000);

    if (halt) begin
      $display("\n--- HALT Asserted at Cycle %0d ---", uut.datapath.cycles_current);
      
      // Check Result based on RISC-V Test Standards
      // x17 (a7) should hold 93 (exit syscall number)
      // x10 (a0) should hold 0 for pass, or failing test number >> 1
      
      if (uut.datapath.rf.regs[17] === 32'd93) begin
        if (uut.datapath.rf.regs[10] === 32'd0) begin
            $display("RESULT: PASS");
        end else begin
            $display("RESULT: FAIL (Error Code: %0d)", uut.datapath.rf.regs[10] >> 1);
        end
      end else begin
        $display("RESULT: UNKNOWN (x17 != 93)");
        $display("x10 (a0): %h", uut.datapath.rf.regs[10]);
        $display("x17 (a7): %h", uut.datapath.rf.regs[17]);
      end
    end else begin
      $display("\n--- TIMEOUT (1000 Cycles reached) ---");
      $display("PC is at: %h", uut.datapath.f_pc);
    end

    $finish;
  end

  // Monitor Trace
  always @(posedge clk) begin
    if (!rst && trace_writeback_pc != 0) begin
      $display("Time: %0t | Cycle: %0d | WB PC: %h | Inst: %h", 
               $time, uut.datapath.cycles_current, trace_writeback_pc, trace_writeback_inst);
    end
  end

  // Task to hardcode the hex content
  task initialize_memory;
    integer i;
    begin
      // Clear memory
      for (i = 0; i < 8192; i = i + 1) begin
        uut.memory.mem_array[i] = 32'b0;
      end

      // Load Block 1 (Address 0x000)
      uut.memory.mem_array[0]  = 32'hffc00113;
      uut.memory.mem_array[1]  = 32'h00000c13;
      uut.memory.mem_array[2]  = 32'h00100413;
      uut.memory.mem_array[3]  = 32'h00100b93;
      uut.memory.mem_array[4]  = 32'hff002cb7;
      uut.memory.mem_array[5]  = 32'hff001937;
      uut.memory.mem_array[6]  = 32'hfdd00a13;
      uut.memory.mem_array[7]  = 32'h08000a93;
      uut.memory.mem_array[8]  = 32'h00100993;
      uut.memory.mem_array[9]  = 32'hf8100b13;
      uut.memory.mem_array[10] = 32'h034b8533;
      uut.memory.mem_array[11] = 32'h008c8023;
      uut.memory.mem_array[12] = 32'h00094483;
      uut.memory.mem_array[13] = 32'h0ff4f493;
      uut.memory.mem_array[14] = 32'h0044f493;
      uut.memory.mem_array[15] = 32'h12c50513;
      uut.memory.mem_array[16] = 32'h1c0000ef;
      uut.memory.mem_array[17] = 32'h00094783;
      uut.memory.mem_array[18] = 32'h0ff7f793;
      uut.memory.mem_array[19] = 32'h04049863;
      uut.memory.mem_array[20] = 32'h0047f793;
      uut.memory.mem_array[21] = 32'h04078463;
      uut.memory.mem_array[22] = 32'h03340463;
      uut.memory.mem_array[23] = 32'hff002437;
      uut.memory.mem_array[24] = 32'hfff00493;
      uut.memory.mem_array[25] = 32'h00940023;
      uut.memory.mem_array[26] = 32'h06400513;
      uut.memory.mem_array[27] = 32'h194000ef;
      uut.memory.mem_array[28] = 32'h06400513;
      uut.memory.mem_array[29] = 32'h00040023;
      uut.memory.mem_array[30] = 32'h188000ef;
      uut.memory.mem_array[31] = 32'hfe9ff06f;
      uut.memory.mem_array[32] = 32'h016c8023;
      uut.memory.mem_array[33] = 32'h0fa00513;
      uut.memory.mem_array[34] = 32'h178000ef;
      uut.memory.mem_array[35] = 32'h001b8b93;
      uut.memory.mem_array[36] = 32'h008c8023;
      uut.memory.mem_array[37] = 32'h001c4c13;
      uut.memory.mem_array[38] = 32'h00c0006f;
      uut.memory.mem_array[39] = 32'hff540ce3;
      uut.memory.mem_array[40] = 32'hff340ae3;
      uut.memory.mem_array[41] = 32'h000c0863;
      uut.memory.mem_array[42] = 32'h00141413;
      uut.memory.mem_array[43] = 32'h0ff47413;
      uut.memory.mem_array[44] = 32'hf79ff06f;
      uut.memory.mem_array[45] = 32'h00145413;
      uut.memory.mem_array[46] = 32'hf71ff06f;

      // Load Block 2 (Address @200 hex = 512 dec bytes = 128 words)
      uut.memory.mem_array[128] = 32'h000f47b7;
      uut.memory.mem_array[129] = 32'h24078793;
      uut.memory.mem_array[130] = 32'h02f50533;
      uut.memory.mem_array[131] = 32'h2ee00793;
      uut.memory.mem_array[132] = 32'h02f55533;
      uut.memory.mem_array[133] = 32'h00000793;
      uut.memory.mem_array[134] = 32'h00f51463;
      uut.memory.mem_array[135] = 32'h00008067;
      uut.memory.mem_array[136] = 32'h00000013;
      uut.memory.mem_array[137] = 32'h00178793;
      uut.memory.mem_array[138] = 32'hff1ff06f;
    end
  endtask

endmodule