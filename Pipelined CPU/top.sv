// Top Level Module
module top(input  logic        MAX10_CLK1_50, 
           input  logic [1:0]  KEY,
		   input  logic [9:0]  SW,
           output logic [9:0]  LEDR
);

	logic clk, reset;
	logic [31:0] PCF, InstrF, ALUResultM, WriteDataM, ReadDataM;
	logic		 MemWriteM;
	
	assign clk = MAX10_CLK1_50;
	assign reset = ~KEY[0];
	assign LEDR = PCF[11:2];
	
	// Instantiate the Pipelined Processor
	riscvpipelined cpu(
			.clk        (MAX10_CLK1_50),
			.reset      (reset),
			.PCF        (PCF),
			.InstrF     (InstrF),
			.ALUResultM (ALUResultM),
			.WriteDataM (WriteDataM),
			.ReadDataM  (ReadDataM),
			.MemWriteM  (MemWriteM)
	);
	
	// Instantiate Memories
	imem instruction_memory(
		.a(PCF),
		.rd(InstrF)
	);
	
	dmem data_memory(
		.clk(MAX10_CLK1_50),
		.we(MemWriteM),
		.a(ALUResultM),
		.wd(WriteDataM),
		.rd(ReadDataM)
	);
endmodule

module riscvpipelined(
    input  logic        clk, reset,
    output logic [31:0] PCF,          
    input  logic [31:0] InstrF,       
    output logic [31:0] ALUResultM,   
    output logic [31:0] WriteDataM,   
    input  logic [31:0] ReadDataM,    
    output logic        MemWriteM     
);

	// Wires & Interconnects
    logic [1:0] ResultSrcD, ResultSrcE, ResultSrcM, ResultSrcW;
    logic       MemWriteD, MemWriteE;
    logic       JumpD, JumpE;
    logic       BranchD, BranchE;
    logic [2:0] ALUControlD, ALUControlE;
    logic       ALUSrcD, ALUSrcE;
    logic       RegWriteD, RegWriteE, RegWriteM, RegWriteW;
    
    logic       StallF, StallD;
    logic       FlushD, FlushE;
    logic [1:0] ForwardAE, ForwardBE;
    logic [4:0] Rs1D, Rs2D, Rs1E, Rs2E;
    logic [4:0] RdE, RdM, RdW;
    logic       PCSrcE;

    // Internal Decode Signals
    logic [31:0] InstrD;
    logic [2:0]  ImmSrcD;

	// Module Instantiations
    controller c (
        .Op(InstrD[6:0]), 
        .Funct3(InstrD[14:12]), 
        .Funct7(InstrD[31:25]),
        .RegWriteD(RegWriteD), 
        .ResultSrcD(ResultSrcD), 
        .MemWriteD(MemWriteD), 
        .JumpD(JumpD), 
        .BranchD(BranchD), 
        .ALUControlD(ALUControlD), 
        .ALUSrcD(ALUSrcD), 
        .ImmSrcD(ImmSrcD)
    );

    datapath dp (
        .clk(clk), .reset(reset),
        .StallF(StallF), .PCF(PCF), .InstrF(InstrF),
        .InstrD(InstrD), 
        .StallD(StallD), .FlushD(FlushD), .ImmSrcD(ImmSrcD),
        .FlushE(FlushE), 
        .ForwardAE(ForwardAE), .ForwardBE(ForwardBE),
        .PCSrcE(PCSrcE),
        .ResultSrcE(ResultSrcE),
        .ALUControlE(ALUControlE), .ALUSrcE(ALUSrcE),
        .RegWriteD(RegWriteD), .ResultSrcD(ResultSrcD), 
        .MemWriteD(MemWriteD), .JumpD(JumpD), .BranchD(BranchD),
		.ALUControlD(ALUControlD),
		.ALUSrcD(ALUSrcD),
        .Rs1D(Rs1D), .Rs2D(Rs2D), .Rs1E(Rs1E), .Rs2E(Rs2E), 
        .RdE(RdE), .RdM(RdM), .RdW(RdW),
        .MemWriteM(MemWriteM), .ALUResultM(ALUResultM), 
        .WriteDataM(WriteDataM), .ReadDataM(ReadDataM),
        .RegWriteW(RegWriteW), .ResultSrcW(ResultSrcW), 
        .RegWriteM(RegWriteM), .ResultSrcM(ResultSrcM)
    );

    hazard_unit hu (
        .Rs1D(Rs1D), .Rs2D(Rs2D),
        .Rs1E(Rs1E), .Rs2E(Rs2E),
        .RdE(RdE), .RdM(RdM), .RdW(RdW),
        .RegWriteM(RegWriteM), .RegWriteW(RegWriteW),
        .ResultSrcE0(ResultSrcE[0]), 
        .PCSrcE(PCSrcE),
        .ForwardAE(ForwardAE), .ForwardBE(ForwardBE),
        .StallF(StallF), .StallD(StallD),
        .FlushD(FlushD), .FlushE(FlushE)
    );

endmodule

// Datapath
module datapath(
    input  logic        clk, reset,
    // Fetch
    input  logic        StallF,
    output logic [31:0] PCF,
    input  logic [31:0] InstrF,
    // Decode
    output logic [31:0] InstrD,
    input  logic        StallD, FlushD,
    input  logic [2:0]  ImmSrcD,
    input  logic        RegWriteD,
    input  logic [1:0]  ResultSrcD,
    input  logic        MemWriteD,
    input  logic        JumpD, BranchD,
    input  logic [2:0]  ALUControlD,
    input  logic        ALUSrcD,
    output logic [4:0]  Rs1D, Rs2D,
    // Execute
    input  logic        FlushE,
    input  logic [1:0]  ForwardAE, ForwardBE,
    output logic [4:0]  Rs1E, Rs2E, RdE,
    output logic        PCSrcE,
    output logic [1:0]  ResultSrcE,
    output logic [2:0]  ALUControlE,
    output logic        ALUSrcE,
    // Memory
    output logic        MemWriteM, 
    output logic [31:0] ALUResultM, WriteDataM, 
    input  logic [31:0] ReadDataM,
    output logic [4:0]  RdM,
    output logic        RegWriteM, 
    output logic [1:0]  ResultSrcM,
    // Writeback
    output logic        RegWriteW,
    output logic [1:0]  ResultSrcW,
    output logic [4:0]  RdW
);
    
    // Internal Signals
    logic [31:0] PCPlus4F, PCNextF;
    logic [31:0] PCD, PCPlus4D;
    logic [31:0] RD1D, RD2D, ImmExtD;
    logic [4:0]  RdD;
    logic [31:0] PCE, ImmExtE, PCPlus4E;
    logic [31:0] RD1E, RD2E;
    logic [31:0] SrcAE, SrcBE, WriteDataE;
    logic [31:0] ALUResultE;
    logic [31:0] PCTargetE;
    logic        ZeroE;
    logic        RegWriteE, MemWriteE, JumpE, BranchE;
    logic [31:0] PCPlus4M;
    logic [31:0] ALUResultW, ReadDataW, PCPlus4W;
    logic [31:0] ResultW;

    // Stage 1: Fetch
    assign PCSrcE = (BranchE & ZeroE) | JumpE;
    assign PCNextF = PCSrcE ? PCTargetE : PCPlus4F;
    flopenr #(32) pcreg(clk, reset, ~StallF, PCNextF, PCF);
    assign PCPlus4F = PCF + 4;

	// Pipeline Register F/D
    always_ff @(posedge clk or posedge reset) begin
        if (reset) begin
            InstrD   <= 0;
            PCD      <= 0;
            PCPlus4D <= 0;
        end else if (~StallD) begin 
            if (FlushD) begin       
                InstrD   <= 0;
                PCD      <= 0;
                PCPlus4D <= 0;
            end else begin
                InstrD   <= InstrF;
                PCD      <= PCF;
                PCPlus4D <= PCPlus4F;
            end
        end
    end

    // Stage 2: Decode
    assign Rs1D = InstrD[19:15];
    assign Rs2D = InstrD[24:20];
    assign RdD  = InstrD[11:7];

    regfile rf (
        .clk(clk), .we3(RegWriteW), 
        .a1(Rs1D), .a2(Rs2D), .a3(RdW), 
        .wd3(ResultW), .rd1(RD1D), .rd2(RD2D)
    );
    extend ext (.instr(InstrD[31:7]), .immsrc(ImmSrcD), .immext(ImmExtD));

	// Pipeline Register D/E
    always_ff @(posedge clk or posedge reset) begin
        if (reset) begin
            RegWriteE   <= 0; ResultSrcE <= 0; MemWriteE <= 0;
            JumpE       <= 0; BranchE    <= 0; ALUControlE <= 0;
            ALUSrcE     <= 0;
            RD1E        <= 0; RD2E       <= 0;
            PCE         <= 0; Rs1E       <= 0; Rs2E        <= 0;
            RdE         <= 0; ImmExtE    <= 0; PCPlus4E    <= 0;
        end else if (FlushE) begin
            RegWriteE   <= 0; ResultSrcE <= 0; MemWriteE <= 0;
            JumpE       <= 0; BranchE    <= 0; ALUControlE <= 0;
            ALUSrcE     <= 0;
            RD1E        <= 0; RD2E       <= 0;
            PCE         <= 0; Rs1E       <= 0; Rs2E        <= 0;
            RdE         <= 0; ImmExtE    <= 0; PCPlus4E    <= 0;
        end else begin
            RegWriteE   <= RegWriteD;
            ResultSrcE  <= ResultSrcD;
            MemWriteE   <= MemWriteD;
            JumpE       <= JumpD;
            BranchE     <= BranchD;
            ALUControlE <= ALUControlD;
            ALUSrcE     <= ALUSrcD;
            RD1E        <= RD1D;
            RD2E        <= RD2D;
            PCE         <= PCD;
            Rs1E        <= Rs1D;
            Rs2E        <= Rs2D;
            RdE         <= RdD;
            ImmExtE     <= ImmExtD;
            PCPlus4E    <= PCPlus4D;
        end
    end

    // Stage 3: Execute
    always_comb begin
        case(ForwardAE)
            2'b00: SrcAE = RD1E;
            2'b01: SrcAE = ResultW;
            2'b10: SrcAE = ALUResultM;
            default: SrcAE = RD1E;
        endcase
        case(ForwardBE)
            2'b00: WriteDataE = RD2E;
            2'b01: WriteDataE = ResultW;
            2'b10: WriteDataE = ALUResultM;
            default: WriteDataE = RD2E;
        endcase
    end

    assign SrcBE = ALUSrcE ? ImmExtE : WriteDataE;
    alu alu_inst (.SrcA(SrcAE), .SrcB(SrcBE), .ALUControl(ALUControlE), .ALUResult(ALUResultE), .Zero(ZeroE));
    assign PCTargetE = PCE + ImmExtE;

	// Pipeline Register E/M
    always_ff @(posedge clk or posedge reset) begin
        if (reset) begin
            RegWriteM   <= 0; ResultSrcM <= 0; MemWriteM <= 0;
            ALUResultM  <= 0; WriteDataM <= 0; RdM       <= 0;
            PCPlus4M    <= 0;
        end else begin
            RegWriteM   <= RegWriteE;
            ResultSrcM  <= ResultSrcE;
            MemWriteM   <= MemWriteE;
            ALUResultM  <= ALUResultE;
            WriteDataM  <= WriteDataE;
            RdM         <= RdE;
            PCPlus4M    <= PCPlus4E;
        end
    end

	// Stage 4: Memory
	// Pipeline Register M/W
    always_ff @(posedge clk or posedge reset) begin
        if (reset) begin
            RegWriteW   <= 0; ResultSrcW <= 0;
            ALUResultW  <= 0; ReadDataW  <= 0; RdW       <= 0;
            PCPlus4W    <= 0;
        end else begin
            RegWriteW   <= RegWriteM;
            ResultSrcW  <= ResultSrcM;
            ALUResultW  <= ALUResultM;
            ReadDataW   <= ReadDataM;
            RdW         <= RdM;
            PCPlus4W    <= PCPlus4M;
        end
    end

    // Stage 5: Writeback
    always_comb begin
        case(ResultSrcW)
            2'b00: ResultW = ALUResultW;
            2'b01: ResultW = ReadDataW;
            2'b10: ResultW = PCPlus4W;
            default: ResultW = ALUResultW;
        endcase
    end
endmodule

// Hazard Unit
module hazard_unit(
    input  logic [4:0] Rs1D, Rs2D, Rs1E, Rs2E, RdE, RdM, RdW,
    input  logic       RegWriteM, RegWriteW,
    input  logic       ResultSrcE0, 
    input  logic       PCSrcE,      
    output logic [1:0] ForwardAE, ForwardBE,
    output logic       StallF, StallD, FlushD, FlushE
);

    logic lwStall;

    // Forwarding
    always_comb begin
        if      ((Rs1E == RdM) && RegWriteM && (Rs1E != 0)) ForwardAE = 2'b10;
        else if ((Rs1E == RdW) && RegWriteW && (Rs1E != 0)) ForwardAE = 2'b01;
        else                                                ForwardAE = 2'b00;

        if      ((Rs2E == RdM) && RegWriteM && (Rs2E != 0)) ForwardBE = 2'b10;
        else if ((Rs2E == RdW) && RegWriteW && (Rs2E != 0)) ForwardBE = 2'b01;
        else                                                ForwardBE = 2'b00;
    end

    // Stalling & Flushing
    assign lwStall = ResultSrcE0 & ((Rs1D == RdE) | (Rs2D == RdE));
    assign StallF = lwStall;
    assign StallD = lwStall;
    assign FlushE = lwStall | PCSrcE;
    assign FlushD = PCSrcE;
endmodule

// Controller
module controller(
    input  logic [6:0] Op,
    input  logic [2:0] Funct3,
    input  logic [6:0] Funct7,
    output logic       RegWriteD,
    output logic [1:0] ResultSrcD,
    output logic       MemWriteD,
    output logic       JumpD,
    output logic       BranchD,
    output logic [2:0] ALUControlD,
    output logic       ALUSrcD,
    output logic [2:0] ImmSrcD
);

    logic [1:0] ALUOp;

    always_comb begin
        case(Op)
            7'b0000011: begin RegWriteD=1; ResultSrcD=2'b01; MemWriteD=0; JumpD=0; BranchD=0; ALUOp=2'b00; ALUSrcD=1;    ImmSrcD=3'b000; end // lw
            7'b0100011: begin RegWriteD=0; ResultSrcD=2'bxx; MemWriteD=1; JumpD=0; BranchD=0; ALUOp=2'b00; ALUSrcD=1;    ImmSrcD=3'b001; end // sw
            7'b0110011: begin RegWriteD=1; ResultSrcD=2'b00; MemWriteD=0; JumpD=0; BranchD=0; ALUOp=2'b10; ALUSrcD=0;    ImmSrcD=3'bxxx; end // R-type
            7'b1100011: begin RegWriteD=0; ResultSrcD=2'bxx; MemWriteD=0; JumpD=0; BranchD=1; ALUOp=2'b01; ALUSrcD=0;    ImmSrcD=3'b010; end // beq
            7'b0010011: begin RegWriteD=1; ResultSrcD=2'b00; MemWriteD=0; JumpD=0; BranchD=0; ALUOp=2'b10; ALUSrcD=1;    ImmSrcD=3'b000; end // I-type
            7'b1101111: begin RegWriteD=1; ResultSrcD=2'b10; MemWriteD=0; JumpD=1; BranchD=0; ALUOp=2'bxx; ALUSrcD=1'bx; ImmSrcD=3'b011; end // jal
            default:    begin RegWriteD=0; ResultSrcD=2'bxx; MemWriteD=0; JumpD=0; BranchD=0; ALUOp=2'b00; ALUSrcD=0;    ImmSrcD=3'b000; end
        endcase
    end

    always_comb begin
        case(ALUOp)
            2'b00: ALUControlD = 3'b000; // Add
            2'b01: ALUControlD = 3'b001; // Subtract
            2'b10: begin 
                case(Funct3)
                    3'b000: if ({Op[5], Funct7[5]} == 2'b11) ALUControlD = 3'b001; // sub
                            else                             ALUControlD = 3'b000; // add
                    3'b010: ALUControlD = 3'b101; // slt
                    3'b110: ALUControlD = 3'b011; // or
                    3'b111: ALUControlD = 3'b010; // and
                    default: ALUControlD = 3'bxxx; 
                endcase
            end
            default: ALUControlD = 3'bxxx;
        endcase
    end
endmodule

// Helper Modules
module regfile(input  logic        clk, 
               input  logic        we3, 
               input  logic [4:0]  a1, a2, a3, 
               input  logic [31:0] wd3, 
               output logic [31:0] rd1, rd2);
    logic [31:0] rf[31:0];
    always_ff @(posedge clk) if (we3) rf[a3] <= wd3;
    assign rd1 = (a1 != 0) ? rf[a1] : 0;
    assign rd2 = (a2 != 0) ? rf[a2] : 0;
endmodule

module extend(input  logic [31:7] instr,
              input  logic [2:0]  immsrc,
              output logic [31:0] immext);
    always_comb
        case(immsrc) 
            3'b000:   immext = {{20{instr[31]}}, instr[31:20]};  
            3'b001:   immext = {{20{instr[31]}}, instr[31:25], instr[11:7]}; 
            3'b010:   immext = {{20{instr[31]}}, instr[7], instr[30:25], instr[11:8], 1'b0}; 
            3'b011:   immext = {{12{instr[31]}}, instr[19:12], instr[20], instr[30:21], 1'b0}; 
            default:  immext = 32'bx; 
        endcase
endmodule

module alu(input  logic [31:0] SrcA, SrcB,
           input  logic [2:0]  ALUControl,
           output logic [31:0] ALUResult,
           output logic        Zero);
    always_comb begin
        case (ALUControl)
            3'b000: ALUResult = SrcA + SrcB; 
            3'b001: ALUResult = SrcA - SrcB; 
            3'b010: ALUResult = SrcA & SrcB; 
            3'b011: ALUResult = SrcA | SrcB; 
            3'b101: ALUResult = (SrcA < SrcB) ? 32'd1 : 32'd0; 
            default: ALUResult = 32'bx;
        endcase
    end
    assign Zero = (ALUResult == 0);
endmodule

module flopenr #(parameter WIDTH = 8)
                (input  logic             clk, reset, en,
                 input  logic [WIDTH-1:0] d, 
                 output logic [WIDTH-1:0] q);
    always_ff @(posedge clk, posedge reset)
        if (reset)   q <= 0;
        else if (en) q <= d;
endmodule

// Memory Modules
module imem(input logic [31:0] a,
			output logic [31:0] rd);

    logic [31:0] RAM[63:0];

    initial
      $readmemh("C:/Users/arvin/AppData/Local/quartus/riscvpipelined/imem.txt", RAM);
    assign rd = RAM[a[31:2]]; // word aligned
endmodule

module dmem(input logic clk, we, 
			input logic [31:0] a, wd,
			output logic [31:0] rd);

    logic [31:0] RAM[63:0];
	
  	initial
      $readmemh("C:/Users/arvin/AppData/Local/quartus/riscvpipelined/dmem.txt", RAM);
  
    assign rd = RAM[a[31:2]]; // word aligned

    always_ff @(posedge clk)
        if (we) RAM[a[31:2]] <= wd;
endmodule