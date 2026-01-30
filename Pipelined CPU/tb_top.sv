`timescale 1ns / 1ps

module tb_top();

    // DUT Signals
    logic        clk;
    logic [1:0]  KEY;  // KEY[0] is reset (Active Low)
    logic [9:0]  SW;   // Unused
    logic [9:0]  LEDR; // Output monitoring on DE10-Lite

    // Instantiate the Top Level
    top dut (
        .MAX10_CLK1_50(clk),
        .KEY(KEY),
        .SW(SW),
        .LEDR(LEDR)
    );

    // Clock Generation (50 MHz = 20ns period)
    initial begin
        clk = 0;
        forever #10 clk = ~clk;
    end

    // Test Sequence
    initial begin
        // Initialize Inputs
        SW = 0;
        KEY[0] = 1;
        
        // Reset Sequence
        $display("SIMULATION START");
        $display("Applying Reset...");
        KEY[0] = 0;
        #50;
        KEY[0] = 1;
        $display("Reset Released. Processor Running...");

        // Run for enough cycles to complete the program
        #1000; 

        // Final Checks
        $display("END OF PROGRAM CHECKS");
        
        // We use hierarchical paths to peek into the register file
        // dut -> cpu -> dp -> rf -> rf
        
        // Check x3 (10 + 5 = 15)
        if (dut.cpu.dp.rf.rf[3] === 15) 
            $display("[PASS] Basic ALU: x3 = %d", dut.cpu.dp.rf.rf[3]);
        else 
            $display("[FAIL] Basic ALU: x3 = %d (Expected 15)", dut.cpu.dp.rf.rf[3]);

        // Check x4 (15 - 5 = 10) - Tests Forwarding
        if (dut.cpu.dp.rf.rf[4] === 10) 
            $display("[PASS] Forwarding: x4 = %d", dut.cpu.dp.rf.rf[4]);
        else 
            $display("[FAIL] Forwarding: x4 = %d (Expected 10)", dut.cpu.dp.rf.rf[4]);

        // Check x5 (Loaded 99 from memory)
        if (dut.cpu.dp.rf.rf[5] === 99) 
            $display("[PASS] Load Word:  x5 = %d", dut.cpu.dp.rf.rf[5]);
        else 
            $display("[FAIL] Load Word:  x5 = %d (Expected 99)", dut.cpu.dp.rf.rf[5]);

        // Check x6 (99 + 99 = 198) - Tests Load-Use Stall
        if (dut.cpu.dp.rf.rf[6] === 198) 
            $display("[PASS] Stall/LU:   x6 = %d", dut.cpu.dp.rf.rf[6]);
        else 
            $display("[FAIL] Stall/LU:   x6 = %d (Expected 198)", dut.cpu.dp.rf.rf[6]);

        // Check x8 (Branch should skip writes of 1 and 2, writing 3)
        if (dut.cpu.dp.rf.rf[8] === 3) 
            $display("[PASS] Branching:  x8 = %d", dut.cpu.dp.rf.rf[8]);
        else 
            $display("[FAIL] Branching:  x8 = %d (Expected 3, Did flush fail?)", dut.cpu.dp.rf.rf[8]);

        $stop;
    end

    // Signal Monitoring
    always @(negedge clk) begin
        if (KEY[0] == 1) begin // Only print when not in reset
            $display("Time: %0t | PCF: %h | InstrD: %h | StallF: %b | FlushE: %b | x3: %d", 
                     $time, dut.cpu.PCF, dut.cpu.InstrD, dut.cpu.StallF, dut.cpu.FlushE, dut.cpu.dp.rf.rf[3]);
        end
    end

endmodule