module tb_top();
    // 1. Declare signals to connect to the DUT (Device Under Test)
    logic        clk;
    logic [9:0]  SW;
    logic [1:0]  KEY;
    logic [31:0] WriteData, DataAdr;
    logic        MemWrite;
    logic [9:0]  LEDR;
    logic [7:0]  HEX0, HEX1, HEX2, HEX3, HEX4, HEX5;

    // 2. Instantiate the Top Module (The Processor)
    // Note: connecting internal 'clk' to 'MAX10_CLK1_50'
    top dut(
        .MAX10_CLK1_50(clk), 
        .SW(SW), 
        .KEY(KEY), 
        .WriteData(WriteData), 
        .Adr(DataAdr),       // <--- CHANGED: Connect .Adr (port) to DataAdr (signal)
        .MemWrite(MemWrite), 
        .LEDR(LEDR), 
        .HEX0(HEX0), .HEX1(HEX1), .HEX2(HEX2), 
        .HEX3(HEX3), .HEX4(HEX4), .HEX5(HEX5)
    );

    // 3. Generate Clock
    // Toggle clock every 5 time units (creates a 10-unit period)
    initial begin
        clk = 0;
        forever #5 clk = ~clk; 
    end

    // 4. Generate Reset
    initial begin
        SW = 0;
        //KEY[1] = 1; // Key is usually active low on DE10 boards, but let's look at your logic:
                    // Your code: assign reset = ~KEY[1];
                    // So KEY[1] = 0 means Reset is ON. KEY[1] = 1 means Reset is OFF.
        
        // Start with Reset Active
        KEY[1] = 0; 
        #100;        // Wait 20 time units
        KEY[1] = 1; // Release Reset
    end

    // 5. Check for Success
    // This block runs on every falling edge of the clock
    always @(negedge clk) begin
        if (MemWrite) begin
            // If the CPU writes the value 25 (0x19) to address 100 (0x64), it passed.
            // This assumes your assembly code ends with: sw x?, 100(x0) where x? = 25
            if (DataAdr === 32'd100 && WriteData === 32'd25) begin
                $display("Simulation Succeeded! Value 25 written to address 100.");
                $stop; // Stop simulation
            end 
            // If it writes anything else, just print it so we can see what's happening
            else if (DataAdr !== 32'hFF200000) begin // Ignore LED writes
                $display("Memory Write: Address = %h, Data = %h", DataAdr, WriteData);
            end
        end
    end
endmodule