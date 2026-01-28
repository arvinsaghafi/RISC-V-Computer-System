# RISC-V-Computer-System
Implementation of a Single-Cycle, Multi-Cycle, and Pipelined Processor in RISC-V.

## Contents
- [Overview](#overview)
- [Part 1: Single Cycle Core](#part-1-single-cycle-core)
- [Part 2: Multi-Cycle Core](#part-2-multi-cycle-core)
  - [Microarchitecture](#microarchitecture)
  - [Finite State Machine](#finite-state-machine)
  - [Verification & Stress Testing](#verification--stress-testing)
  - [Key Challenges & Debugging](#key-challenges--debugging)
- [Part 3: Pipelined Core](#part-3-pipelined-core)
- [Usage & Simulation](#usage--simulation)

## Overview

This project implements a soft-core processor based on the RISC-V Instruction Set Architecture (ISA). The design process is broken down into three iterations to explore the trade-offs between CPI (Cycles Per Instruction), clock frequency, and hardware complexity.
- ISA: RV32I (Base Integer Instruction Set)
- HDL: SystemVerilog
- Simulation Tools: ModelSim / QuestaSim
- Hardware Target: Intel MAX10 FPGA (Verified via simulation)

## Part 1: Single Cycle Core


## Part 2: Multi-Cycle Core

Status: Verified & Benchmarked

In the second iteration, the processor was refactored into a Multi-Cycle Microarchitecture. Unlike the single-cycle implementation, this design breaks instruction execution into multiple shorter clock cycles (ranging from 3 to 5 cycles per instruction).

### Microarchitecture
The multi-cycle design reuses hardware resources (ALU, Memory) across different clock cycles, significantly reducing the hardware area compared to a single-cycle design.
- **Unified Memory:** A single memory module is used for both Instructions and Data, arbitrated by the control unit.
- **Resource Sharing:** The ALU calculates both data results and memory addresses.
- **State Registers:** Added non-architectural registers (A, B, ALUOut, IR) to hold signals stable between clock cycles.

### Finite State Machine
The core is driven by a complex FSM in the Control Unit that orchestrates the datapath through the following stages:
1. **Fetch:** Load instruction from memory to IR; PC = PC + 4.
2. **Decode:** Read register file; Decode Opcode/Funct fields.
3. **Execute:** Perform ALU operation or calculate branch target.
4. **Memory:** Access memory (Load/Store) if required.
5. **Writeback:** Write result to Register File.

### Verification & Stress Testing
To validate the processor, a comprehensive Stress Test Suite was developed in assembly. This program tests Arithmetic, Logic, Control Flow, and Memory operations in a single continuous run.

**The Stress Test Assembly**
```
// STAGE 1: ARITHMETIC TEST
00500093  // addi x1, x0, 5      ; Init x1 = 5
00a00113  // addi x2, x0, 10     ; Init x2 = 10
002081b3  // add  x3, x1, x2     ; x3 = 5 + 10 = 15
40118233  // sub  x4, x3, x1     ; x4 = 15 - 5 = 10
0020a2b3  // slt  x5, x1, x2     ; x5 = 1 (Since 5 < 10)
// STAGE 2: BRANCH TEST
00028863  // beq  x5, x0, fail   ; Check: Is x5 == 0? (Should be false). If broken, it branches to 'fail' (Trap 1).
00220463  // beq  x4, x2, pass1  ; Check: Is 10 == 10? (Should be true). If working, skips the next instruction.
00100e13  // addi x28, x0, 1     ; [TRAP 1] If we execute this, Branch failed.
// STAGE 3: MEMORY TEST
0541aa23  // sw   x3, 84(x0)     ; pass1: Store 15 (x3) to Mem[84].
05402303  // lw   x6, 84(x0)     ; Load Mem[84] back into x6.
00330463  // beq  x6, x3, pass2  ; Check: Does Loaded(15) == Stored(15)? If working, skips the next instruction.
00200e13  // addi x28, x0, 2     ; [TRAP 2] If we execute this, Memory failed.
// STAGE 4: JUMP TEST
0080006f  // jal  x0, pass3      ; pass2: Unconditional Jump to 'pass3'. Skips the next instruction.
00300e13  // addi x28, x0, 3     ; [TRAP 3] If we execute this, JAL failed.
// STAGE 5: FINISH LINE
01900513  // addi x10, x0, 25    ; pass3: Load "Magic Number" 25 into x10.
06a02223  // sw   x10, 100(x0)   ; Store 25 to Address 100. Testbench looks for this write.
```

**Simulation Results:**

The testbench successfully detected the write of value `25` (0x19) to address `100` (0x64), confirming all logic blocks are functional.

<img width="2447" height="706" alt="riscvmulti_testbench_questa_wave" src="https://github.com/user-attachments/assets/d95b4782-18fa-4bf9-ba6a-de1c13f5bf58" />

*Figure 1: Final waveform verification showing the "Magic Number" 0x19 being written to address 0x64 at the end of the simulation.*

### Key Challenges & Debugging
During implementation, a critical **Race Condition** was identified where the Processor and Memory modules were attempting to drive/read the Address Bus on the same clock edge, leading to metastable values (`Address = 63`).

**Solution:** The memory write logic was shifted to the **negative clock edge** (`negedge clk`). This ensures that the Address and Data signals from the processor (updated on `posedge`) are stable before the memory commits the write.

```
// Fix applied to Memory Module to prevent Race Conditions
always_ff @(negedge clk) 
    if (we) RAM[a[31:2]] <= wd;
```

## Part 3: Pipelined Core




## Usage & Simulation
To reproduce the verification results using ModelSim or QuestaSim:

**1. Clone the repository:**
```
git clone https://github.com/arvinsaghafi/RISC-V-Computer-System.git
cd riscv-computer-system
```
**2. Compile the Design:**
```
vlib work
vlog top.sv testbench.sv
```
**3. Run the Simulation:**
```
vsim -voptargs=+acc work.testbench
run -all
```
**4. Verify Output:**

The transcript should display:
`The transcript should display: Simulation Succeeded! Value 25 written to address 100.`
