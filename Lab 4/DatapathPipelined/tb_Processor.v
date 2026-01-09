`timescale 1ns / 1ps

module tb_Processor;

    // Inputs
    reg clk;
    reg rst;

    // Outputs
    wire halt;
    wire [31:0] trace_writeback_pc;
    wire [31:0] trace_writeback_inst;

    // Instantiate the Unit Under Test (UUT)
    Processor uut (
        .clk(clk), 
        .rst(rst), 
        .halt(halt), 
        .trace_writeback_pc(trace_writeback_pc), 
        .trace_writeback_inst(trace_writeback_inst)
    );

    // Clock Generation (100MHz)
    initial begin
        clk = 0;
        forever #5 clk = ~clk;
    end

    // =========================================================
    // MANUAL MEMORY LOADING (Replaces mem_initial_contents.hex)
    // =========================================================
    initial begin
        // We wait 1ns to ensure the array is initialized by the module first
        #1; 
        
        // --- BLOCK 1: Main Instructions (Starting at Index 0) ---
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

        // --- BLOCK 2: Exception/Jump Handler (Starting at Index 0x200 / 512) ---
        // Your hex file had "@200" which usually means memory index 200 (hex)
        uut.memory.mem_array['h200] = 32'h000f47b7;
        uut.memory.mem_array['h201] = 32'h24078793;
        uut.memory.mem_array['h202] = 32'h02f50533;
        uut.memory.mem_array['h203] = 32'h2ee00793;
        uut.memory.mem_array['h204] = 32'h02f55533;
        uut.memory.mem_array['h205] = 32'h00000793;
        uut.memory.mem_array['h206] = 32'h00f51463;
        uut.memory.mem_array['h207] = 32'h00008067;
        uut.memory.mem_array['h208] = 32'h00000013;
        uut.memory.mem_array['h209] = 32'h00178793;
        uut.memory.mem_array['h20a] = 32'hff1ff06f;
        
        $display("Memory Loaded via Testbench Backdoor.");
    end

    // Test Sequence
    initial begin
        // 1. Initialize Inputs
        rst = 1;

        // 2. Hold Reset for 100 ns
        #100;
        rst = 0;
        
        $display("Simulation Started. Reset released.");

        // 4. Run until halt or timeout
        wait(halt == 1);
        
        #50; // Wait for writeback to complete
        
        $display("--------------------------------------------------");
        $display("Processor Halted at PC: %h", trace_writeback_pc);
        $display("--------------------------------------------------");
        $finish;
    end

    // Timeout Failsafe
    initial begin
        #100000; 
        $display("Error: Simulation timed out! Halt signal never went high.");
        $finish;
    end

endmodule