`timescale 1ns / 1ps


module testbench;

    // Testbench signals to connect to the Processor
    reg  clock_proc;
    reg  clock_mem;
    reg  rst;
    wire halt;

    // Instantiate the Processor (your Device Under Test)
    Processor uut (
        .clock_proc(clock_proc),
        .clock_mem(clock_mem),
        .rst(rst),
        .halt(halt)
    );

    // Generate the main processor clock (4ns period -> 250 MHz)
    initial begin
        clock_proc = 0;
        forever #2 clock_proc = ~clock_proc; // 2ns half-period
    end

    // Generate the memory clock, phase-shifted by 90 degrees (1ns delay)
    initial begin
        clock_mem = 0;
        #1; // Phase shift by 1ns
        forever #2 clock_mem = ~clock_mem;
    end

    // Simulation sequence
    initial begin
        // 1. Start with the processor in a reset state
        rst = 1;
        
        // 2. Wait for a moment before loading memory to avoid race conditions
        #1;

        // --- DIRECTLY INITIALIZE THE PROCESSOR'S MEMORY ---
        // This uses a hierarchical reference to access the mem_array inside the design.
        // Path: testbench -> uut (Processor) -> memory (MemorySingleCycle) -> mem_array
        $display("Loading program memory into the design...");
        uut.memory.mem_array[0]  = 32'hffc00513;
        uut.memory.mem_array[1]  = 32'h00050113;
        uut.memory.mem_array[2]  = 32'h00000097;
        uut.memory.mem_array[3]  = 32'h038080e7;
        // The '@40' from the hex file means we jump to address 0x40 (64)
        uut.memory.mem_array[64] = 32'hff010113;
        uut.memory.mem_array[65] = 32'h00812623;
        uut.memory.mem_array[66] = 32'h00000513;
        uut.memory.mem_array[67] = 32'h00100593;
        uut.memory.mem_array[68] = 32'hff002637;
        uut.memory.mem_array[69] = 32'hff0016b7;
        uut.memory.mem_array[70] = 32'hfde9f737;
        uut.memory.mem_array[71] = 32'h14070713;
        uut.memory.mem_array[72] = 32'h11e1a7b7;
        uut.memory.mem_array[73] = 32'h30078793;
        uut.memory.mem_array[74] = 32'h2ee00813;
        uut.memory.mem_array[75] = 32'h08000893;
        uut.memory.mem_array[76] = 32'h08100293;
        uut.memory.mem_array[77] = 32'h00051337;
        uut.memory.mem_array[78] = 32'h61530313;
        uut.memory.mem_array[79] = 32'h057623b7;
        uut.memory.mem_array[80] = 32'h9f138393;
        uut.memory.mem_array[81] = 32'h00100e13;
        uut.memory.mem_array[82] = 32'h00100e93;
        uut.memory.mem_array[83] = 32'h00c0006f;
        uut.memory.mem_array[84] = 32'h018e1e13;
        uut.memory.mem_array[85] = 32'h019e5e13;
        uut.memory.mem_array[86] = 32'h01c60023;
        uut.memory.mem_array[87] = 32'h0006cf03;
        uut.memory.mem_array[88] = 32'h02ee8fb3;
        uut.memory.mem_array[89] = 32'h00ff8fb3;
        uut.memory.mem_array[90] = 32'h010fec63;
        uut.memory.mem_array[91] = 32'h027fbfb3;
        uut.memory.mem_array[92] = 32'h004fdf93;
        uut.memory.mem_array[93] = 32'hffff8f93;
        uut.memory.mem_array[94] = 32'h0100000f;
        uut.memory.mem_array[95] = 32'hfe0f9ce3;
        uut.memory.mem_array[96] = 32'h0006cf83;
        uut.memory.mem_array[97] = 32'h004f7413;
        uut.memory.mem_array[98] = 32'h0ffe7f13;
        uut.memory.mem_array[99] = 32'h02041663;
        uut.memory.mem_array[100] = 32'h004fff93;
        uut.memory.mem_array[101] = 32'h020f8263;
        uut.memory.mem_array[102] = 32'h02bf1e63;
        uut.memory.mem_array[103] = 32'h00560023;
        uut.memory.mem_array[104] = 32'h00030f93;
        uut.memory.mem_array[105] = 32'hffff8f93;
        uut.memory.mem_array[106] = 32'h0100000f;
        uut.memory.mem_array[107] = 32'hfe0f9ce3;
        uut.memory.mem_array[108] = 32'h001e8e93;
        uut.memory.mem_array[109] = 32'h00b60023;
        uut.memory.mem_array[110] = 32'h00bf0463;
        uut.memory.mem_array[111] = 32'h011f1463;
        uut.memory.mem_array[112] = 32'h00154513;
        uut.memory.mem_array[113] = 32'h00157f13;
        uut.memory.mem_array[114] = 32'hf80f04e3;
        uut.memory.mem_array[115] = 32'h001e1e13;
        uut.memory.mem_array[116] = 32'hf89ff06f;
        uut.memory.mem_array[117] = 32'hff002537;
        uut.memory.mem_array[118] = 32'hfff00593;
        uut.memory.mem_array[119] = 32'h00021637;
        uut.memory.mem_array[120] = 32'h8d560613;
        uut.memory.mem_array[121] = 32'h00b50023;
        uut.memory.mem_array[122] = 32'h00060693;
        uut.memory.mem_array[123] = 32'hfff68693;
        uut.memory.mem_array[124] = 32'h0100000f;
        uut.memory.mem_array[125] = 32'hfe069ce3;
        uut.memory.mem_array[126] = 32'h00050023;
        uut.memory.mem_array[127] = 32'h00060693;
        uut.memory.mem_array[128] = 32'hfff68693;
        uut.memory.mem_array[129] = 32'h0100000f;
        uut.memory.mem_array[130] = 32'hfe069ce3;
        uut.memory.mem_array[131] = 32'hfd9ff06f;
        $display("Memory loading complete.");
        
        // 3. Wait for a few clock cycles with reset asserted
        #20;

        // 4. De-assert the reset to begin program execution
        $display("De-asserting reset. Starting processor.");
        rst = 0;

        // 5. Let the simulation run for a while, or until halt
        // For a long program, you might need to increase this timeout.
        #5000;

        // 6. End the simulation
        $display("Simulation timed out or finished.");
        $finish;
    end

endmodule