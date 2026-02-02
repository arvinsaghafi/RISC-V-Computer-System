module top(input  logic        MAX10_CLK1_50, 
           input  logic [9:0]  SW,
           input  logic [1:0]  KEY,
           output logic [31:0] WriteData, 
           output logic [31:0] Adr, // Renamed from DataAdr to Adr (shared address)
           output logic        MemWrite,
           output logic [9:0]  LEDR,
           output logic [7:0]  HEX0, HEX1, HEX2, HEX3, HEX4, HEX5);

    logic clk, reset;
    assign clk = MAX10_CLK1_50;
    assign reset = ~KEY[1];

    logic [31:0] ReadData;
    logic [31:0] MemData; // Data from memory to processor

    logic [15:0] HEX5HEX4;
    logic [31:0] HEX3HEX0;

    assign {HEX3, HEX2, HEX1, HEX0} = ~HEX3HEX0;
    assign {HEX5, HEX4} = ~HEX5HEX4;

    // Instantiate multi-cycle processor
    riscvmulti rvmulti(clk, reset, ReadData, MemWrite, Adr, WriteData);

    // Instantiate unified memory
    // Multi-cycle uses one memory for instructions and data
    mem unified_mem(clk, MemWrite, Adr, WriteData, MemData);

    // Mux for I/O (memory mapped I/O)
    // We intercept the ReadData going back to the processor
    always_comb begin
        if (Adr == 32'hFF200040)
            ReadData = {22'b0, SW};   // Switches
        else
            ReadData = MemData;       // Main memory
    end

    // Memory mapped output (LEDs and HEX)
    always_ff @(posedge clk) begin
        if (reset) begin
            LEDR <= 10'b0;
            HEX3HEX0 <= 32'b0;
            HEX5HEX4 <= 16'b0;
        end
        else if (MemWrite) begin
            case(Adr)
                32'hFF200000: LEDR <= WriteData[9:0];
                32'hFF200020: HEX3HEX0 <= WriteData;
                32'hFF200030: HEX5HEX4 <= WriteData[15:0];
            endcase
        end
    end
endmodule

module riscvmulti(input  logic        clk, reset,
                  input	 logic [31:0] ReadData,   // Data to datapath (after I/O mux)
                  output logic        MemWrite,
                  output logic [31:0] Adr,
                  output logic [31:0] WriteData);

    logic       Zero;
    logic       PCWrite, AdrSrc, IRWrite;
    logic       RegWrite;
    logic [1:0] ResultSrc;
    logic [1:0] ALUSrcA, ALUSrcB, ImmSrc;
    logic [2:0] ALUControl;
    logic [31:0] Instr; // Instruction from IR

    controller c(clk, reset,
                 Instr[6:0], Instr[14:12], Instr[30], // Op, Funct3, Funct7
                 Zero,
                 PCWrite, AdrSrc, MemWrite, IRWrite,
                 ResultSrc, ALUSrcA, ALUSrcB, ImmSrc, RegWrite, ALUControl);

    datapath dp(clk, reset,
                PCWrite, AdrSrc, IRWrite,
                ResultSrc, ALUSrcA, ALUSrcB, ImmSrc, RegWrite, ALUControl,
                ReadData,          // Input from Memory or I/O system
                Zero,
                Adr, WriteData,    // Outputs to memory
                Instr);            // Output instruction to controller
endmodule

module controller(input  logic       clk, reset,
                  input  logic [6:0] op,
                  input  logic [2:0] funct3,
                  input  logic       funct7b5,
                  input  logic       Zero,
                  output logic       PCWrite, AdrSrc, MemWrite, IRWrite,
                  output logic [1:0] ResultSrc,
                  output logic [1:0] ALUSrcA, ALUSrcB, 
                  output logic [1:0] ImmSrc,
                  output logic       RegWrite,
                  output logic [2:0] ALUControl);

    logic [1:0] ALUOp;
    logic       Branch, PCUpdate;

    // Combinational instruction decoder for ImmSrc
    always_comb case(op)
        7'b0000011: ImmSrc = 2'b00; // lw (I-type)
        7'b0100011: ImmSrc = 2'b01; // sw (S-type)
        7'b0110011: ImmSrc = 2'bxx; // R-type
        7'b1100011: ImmSrc = 2'b10; // beq (B-type)
        7'b0010011: ImmSrc = 2'b00; // I-type ALU
        7'b1101111: ImmSrc = 2'b11; // jal (J-type)
        default:    ImmSrc = 2'bxx;
    endcase

    // Main FSM to control signals per cycle
    mainfsm fsm(clk, reset, op, 
                PCUpdate, Branch, RegWrite, MemWrite, IRWrite,
                AdrSrc, ResultSrc, ALUSrcA, ALUSrcB, ALUOp);

    // ALU Decoder
    aludec ad(op[5], funct3, funct7b5, ALUOp, ALUControl);

    // PC Write logic
    // PC updates if JAL/Fetch (PCUpdate) OR branch taken (Branch & Zero)
    assign PCWrite = PCUpdate | (Branch & Zero);

endmodule

module mainfsm(input  logic       clk, reset,
               input  logic [6:0] op,
               output logic       PCUpdate, Branch, RegWrite, MemWrite, IRWrite,
               output logic       AdrSrc,
               output logic [1:0] ResultSrc,
               output logic [1:0] ALUSrcA, ALUSrcB,
               output logic [1:0] ALUOp);

    // State definition
    typedef enum logic [3:0] {
        FETCH    = 4'd0,  // S0
        DECODE   = 4'd1,  // S1
        MEMADR   = 4'd2,  // S2
        MEMREAD  = 4'd3,  // S3
        MEMWB    = 4'd4,  // S4
        MEMWRITE = 4'd5,  // S5
        EXECR    = 4'd6,  // S6
        ALUWB    = 4'd7,  // S7
        EXECI    = 4'd8,  // S8
        JAL      = 4'd9,  // S9
        BEQ      = 4'd10  // S10
    } statetype;

    statetype state, nextstate;

    // State register
    always_ff @(posedge clk or posedge reset)
        if (reset) state <= FETCH;
        else       state <= nextstate;

    // Next state logic
    always_comb begin
        case(state)
            FETCH:   nextstate = DECODE;
            DECODE:  case(op)
                        7'b0000011: nextstate = MEMADR; // lw
                        7'b0100011: nextstate = MEMADR; // sw
                        7'b0110011: nextstate = EXECR;  // R-type
                        7'b0010011: nextstate = EXECI;  // I-type ALU
                        7'b1101111: nextstate = JAL;    // jal
                        7'b1100011: nextstate = BEQ;    // beq
                        default:    nextstate = FETCH;
                     endcase
            MEMADR:  case(op)
                        7'b0000011: nextstate = MEMREAD; // lw
                        7'b0100011: nextstate = MEMWRITE; // sw
                        default:    nextstate = FETCH;
                     endcase
            MEMREAD: nextstate = MEMWB;
            MEMWB:   nextstate = FETCH;
            MEMWRITE:nextstate = FETCH;
            EXECR:   nextstate = ALUWB;
            ALUWB:   nextstate = FETCH;
            EXECI:   nextstate = ALUWB;
            JAL:     nextstate = ALUWB;
            BEQ:     nextstate = FETCH;
            default: nextstate = FETCH;
        endcase
    end

    // Output logic
    always_comb begin
        PCUpdate = 0; Branch = 0; RegWrite = 0; MemWrite = 0; IRWrite = 0;
        AdrSrc = 0; ResultSrc = 2'b00; ALUSrcA = 2'b00; ALUSrcB = 2'b00; ALUOp = 2'b00;

        case(state)
            FETCH: begin // S0
                AdrSrc = 0;         // Adr = PC
                IRWrite = 1;        // Write IR
                ALUSrcA = 2'b00;    // PC
                ALUSrcB = 2'b10;    // 4
                ALUOp = 2'b00;      // Add
                ResultSrc = 2'b10;  // Result = ALUResult (PC+4)
                PCUpdate = 1;       // Update PC
            end
            DECODE: begin // S1
                ALUSrcA = 2'b01;    // OldPC
                ALUSrcB = 2'b01;    // Imm
                ALUOp = 2'b00;      // Add (Computes Target)
            end
            MEMADR: begin // S2
                ALUSrcA = 2'b10;    // rs1
                ALUSrcB = 2'b01;    // Imm
                ALUOp = 2'b00;      // Add (Computes effective address)
            end
            MEMREAD: begin // S3
                ResultSrc = 2'b00;  // Result = ALUOut
                AdrSrc = 1;         // Adr = Result
            end
            MEMWB: begin // S4
                ResultSrc = 2'b01;  // Result = MDR
                RegWrite = 1;       // Write to RegFile
            end
            MEMWRITE: begin // S5
                ResultSrc = 2'b00;  // Result = ALUOut
                AdrSrc = 1;         // Adr = Result
                MemWrite = 1;       // Write Memory
            end
            EXECR: begin // S6
                ALUSrcA = 2'b10;    // rs1
                ALUSrcB = 2'b00;    // rs2
                ALUOp = 2'b10;      // R-type op
            end
            ALUWB: begin // S7
                ResultSrc = 2'b00;  // Result = ALUOut
                RegWrite = 1;       // Write to RegFile
            end
            EXECI: begin // S8
                ALUSrcA = 2'b10;    // rs1
                ALUSrcB = 2'b01;    // Imm
                ALUOp = 2'b10;      // I-type op
            end
            JAL: begin // S9
                ALUSrcA = 2'b01;    // OldPC
                ALUSrcB = 2'b10;    // 4
                ALUOp = 2'b00;      // Add (Computes PC+4)
                ResultSrc = 2'b00;  // Result = ALUOut (contains target from S1)
                PCUpdate = 1;       // PC = Result (Target)
                // ALUOut updates to PC+4 at the end of this cycle
                // Next state S7 will write this new ALUOut to RegFile
            end
            BEQ: begin // S10
                ALUSrcA = 2'b10;    // rs1
                ALUSrcB = 2'b00;    // rs2
                ALUOp = 2'b01;      // Sub
                ResultSrc = 2'b00;  // Result = ALUOut (contains Target from S1)
                Branch = 1;         // PC = Result if Zero
            end
        endcase
    end
endmodule

module aludec(input  logic       opb5,
              input  logic [2:0] funct3,
              input  logic       funct7b5,
              input  logic [1:0] ALUOp,
              output logic [2:0] ALUControl);
    logic RtypeSub;
    assign RtypeSub = funct7b5 & opb5; 

    always_comb
        case(ALUOp)
            2'b00: ALUControl = 3'b000; // addition
            2'b01: ALUControl = 3'b001; // subtraction
            default: case(funct3) 
                3'b000: if (RtypeSub) ALUControl = 3'b001; // sub
                        else          ALUControl = 3'b000; // add
                3'b010: ALUControl = 3'b101; // slt
                3'b110: ALUControl = 3'b011; // or
                3'b111: ALUControl = 3'b010; // and
                default: ALUControl = 3'bxxx;
            endcase
        endcase
endmodule

module datapath(input  logic        clk, reset,
                input  logic        PCWrite, AdrSrc, IRWrite,
                input  logic [1:0]  ResultSrc,
                input  logic [1:0]  ALUSrcA, ALUSrcB,
                input  logic [1:0]  ImmSrc,
                input  logic        RegWrite,
                input  logic [2:0]  ALUControl,
                input  logic [31:0] ReadData,
                output logic        Zero,
                output logic [31:0] Adr, WriteData,
                output logic [31:0] Instr);

    logic [31:0] PC, PCNext, OldPC;
    logic [31:0] Data;
    logic [31:0] RD1, RD2;
    logic [31:0] A, B;
    logic [31:0] ImmExt;
    logic [31:0] SrcA, SrcB;
    logic [31:0] ALUResult, ALUOut;
    logic [31:0] Result;

    // PC logic
    flopenr #(32) pcreg(clk, reset, PCWrite, Result, PC);
    
    // Memory address mux
    mux2 #(32) adrmux(PC, Result, AdrSrc, Adr);
    
    // Instruction register (Enable = IRWrite) and OldPC
    flopenr #(32) instrreg(clk, reset, IRWrite, ReadData, Instr);
    flopenr #(32) oldpcreg(clk, reset, IRWrite, PC, OldPC);

    // Data register (MDR) to capture data from memory
    flopr #(32) datareg(clk, reset, ReadData, Data);

    // Register file
    // Reads addresses from instr register. Writes from result.
    regfile rf(clk, RegWrite, Instr[19:15], Instr[24:20], Instr[11:7], 
               Result, RD1, RD2);
    
    // Extend unit
    extend ext(Instr[31:7], ImmSrc, ImmExt);

    // Operand registers A and B
    flopr #(32) areg(clk, reset, RD1, A);
    flopr #(32) breg(clk, reset, RD2, B);
    
    // WriteData for store is B
    assign WriteData = B;

    // ALU source muxes
    // SrcA: 00=PC, 01=OldPC, 10=A
    mux3 #(32) srcamux(PC, OldPC, A, ALUSrcA, SrcA);
    
    // SrcB: 00=B, 01=ImmExt, 10=4
    mux3 #(32) srcbmux(B, ImmExt, 32'd4, ALUSrcB, SrcB);

    // ALU
    alu alu(SrcA, SrcB, ALUControl, ALUResult, Zero);

    // ALU output register
    flopr #(32) aluoutreg(clk, reset, ALUResult, ALUOut);

    // 11. Result mux
    // 00: ALUOut, 01: Data, 10: ALUResult, 11: ImmExt
    mux4 #(32) resultmux(ALUOut, Data, ALUResult, ImmExt, ResultSrc, Result);

endmodule

module extend(input  logic [31:7] instr,
              input  logic [1:0]  immsrc,
              output logic [31:0] immext);
    always_comb
        case(immsrc)
            2'b00: immext = {{20{instr[31]}}, instr[31:20]}; // I-type
            2'b01: immext = {{20{instr[31]}}, instr[31:25], instr[11:7]}; // S-type
            2'b10: immext = {{20{instr[31]}}, instr[7], instr[30:25], instr[11:8], 1'b0}; // B-type
            2'b11: immext = {{12{instr[31]}}, instr[19:12], instr[20], instr[30:21], 1'b0}; // J-type
            default: immext = 32'bx;
        endcase
endmodule

module regfile(input  logic        clk,
               input  logic        we3,
               input  logic [4:0]  a1, a2, a3,
               input  logic [31:0] wd3,
               output logic [31:0] rd1, rd2);
    logic [31:0] rf[31:0];
    always_ff @(posedge clk)
        if (we3) rf[a3] <= wd3;
    assign rd1 = (a1 != 0) ? rf[a1] : 0;
    assign rd2 = (a2 != 0) ? rf[a2] : 0;
endmodule

module alu(input  logic [31:0] a, b,
           input  logic [2:0]  alucontrol,
           output logic [31:0] result,
           output logic        zero);
    logic [31:0] condinvb, sum;
    logic v;      // overflow
    logic isAddSub;       // true when is add or sub

    assign condinvb = alucontrol[0] ? ~b : b;
    assign sum = a + condinvb + alucontrol[0];
    assign isAddSub = ~alucontrol[2] & ~alucontrol[1] |
                      ~alucontrol[1] & alucontrol[0];

    always_comb
        case(alucontrol)
            3'b000:  result = sum;         // add
            3'b001:  result = sum;         // sub
            3'b010:  result = a & b;       // and
            3'b011:  result = a | b;       // or
            3'b100:  result = a ^ b;       // xor
            3'b101:  result = sum[31] ^ v; // slt
            3'b110:  result = a << b[4:0]; // sll
            3'b111:  result = a >> b[4:0]; // srl
            default: result = 32'bx;
        endcase

    assign zero = (result == 32'b0);
    assign v = ~(alucontrol[0] ^ a[31] ^ b[31]) & (a[31] ^ sum[31]) & isAddSub;
endmodule

module mem(input  logic        clk, we,
           input  logic [31:0] a, wd,
           output logic [31:0] rd);
    // Unified memory for Instruction and Data
    logic [31:0] RAM[63:0]; 
    
    // Load your program here
    initial
        $readmemh("mem.txt", RAM); // Change filename as needed

    assign rd = RAM[a[31:2]]; // word aligned read

    always_ff @(posedge clk)
        if (we) RAM[a[31:2]] <= wd;
endmodule

module flopr #(parameter WIDTH = 8)
    (input  logic             clk, reset,
     input  logic [WIDTH-1:0] d,
     output logic [WIDTH-1:0] q);
    always_ff @(posedge clk, posedge reset)
        if (reset) q <= 0;
        else       q <= d;
endmodule

module flopenr #(parameter WIDTH = 8)
    (input  logic             clk, reset, en,
     input  logic [WIDTH-1:0] d,
     output logic [WIDTH-1:0] q);
    always_ff @(posedge clk, posedge reset)
        if (reset)   q <= 0;
        else if (en) q <= d;
endmodule

module mux2 #(parameter WIDTH = 8)
    (input  logic [WIDTH-1:0] d0, d1,
     input  logic             s,
     output logic [WIDTH-1:0] y);
    assign y = s ? d1 : d0;
endmodule

module mux3 #(parameter WIDTH = 8)
    (input  logic [WIDTH-1:0] d0, d1, d2,
     input  logic [1:0]       s,
     output logic [WIDTH-1:0] y);
    assign y = s[1] ? d2 : (s[0] ? d1 : d0);
endmodule

module mux4 #(parameter WIDTH = 8)
    (input  logic [WIDTH-1:0] d0, d1, d2, d3,
     input  logic [1:0]       s,
     output logic [WIDTH-1:0] y);
    always_comb
        case(s)
            2'b00: y = d0;
            2'b01: y = d1;
            2'b10: y = d2;
            2'b11: y = d3;
        endcase
endmodule