# RISC-V Computer System

![SystemVerilog](https://img.shields.io/badge/Language-SystemVerilog-181717?style=for-the-badge&logo=systemverilog)
![ISA](https://img.shields.io/badge/ISA-RISC--V_RV32I-blue?style=for-the-badge)

A comprehensive implementation of the RISC-V (RV32I) architecture in SystemVerilog. This project explores the evolution of processor microarchitecture by iterating through three distinct designs: **Single-Cycle**, **Multi-Cycle**, and **Pipelined**. The core is designed to be synthesizable for Intel FPGAs and has been verified using a custom assembly stress-test suite in ModelSim.

## Contents
- [Overview](#overview)
- [System Specifications](#system-specifications)
- [Part 1: Single-Cycle Core](#part-1-single-cycle-core)
- [Part 2: Multi-Cycle Core](#part-2-multi-cycle-core)
  - [Microarchitecture & FSM](#microarchitecture--fsm)
  - [Verification & Stress Testing](#verification--stress-testing)
  - [Key Challenge: Race Conditions](#key-challenge-race-conditions)
- [Part 3: Pipelined Core](#part-3-pipelined-core)
  - [Hazard Handling](#hazard-handling)
  - [Verification: Hazard Stress Test](#verification-hazard-stress-test)
- [Usage & Simulation](#usage--simulation)
- [References](#references)

## Overview

This project implements a soft-core processor based on the RISC-V Instruction Set Architecture (ISA). The design process serves as a study in hardware/software co-design, analyzing the trade-offs between CPI (Cycles Per Instruction), Clock Frequency, and Hardware Area.
- **Iteration 1 (Single-Cycle)**: Focuses on simplicity and atomic instruction execution.
- **Iteration 2 (Multi-Cycle)**: Focuses on area efficiency and higher clock speeds by breaking critical paths.
- **Iteration 3 (Pipelined)**: Focuses on throughput optimization using instruction-level parallelism and hazard mitigation.

| Feature         | Details                                     |
| :---            | :---                                        |
| ISA             | RISC-V RV32I (Base Integer Instruction Set) |
| HDL             | SystemVerilog (IEEE 1800-2017)              |
| Simulation      | ModelSim / QuestaSim                        |
| Synthesis       | Intel Quartus Prime Lite                    |
| Hardware Target | Intel MAX10 FPGA                            |

## Part 1: Single Cycle Core

The first iteration implements the entire instruction execution in a single clock cycle. While conceptually simple, the clock period is limited by the longest path in the circuit (Load Word instruction).

<p align="center">
  <img src="Single-Cycle%20CPU/Single_Cycle_CPU_Schematic.svg" alt="Single Cycle Schematic" width="800">
  <br>
  <em>Figure 1: Complete Single-Cycle Processor. Adapted from [1].</em>
</p>

- **CPI:** 1 (Theoretical)
- **Architecture:** Harward (Separate Instruction and Data Memory).
- **Limitation:** The clock cycle must be long enough to accommodate the slowest instruction (LW), leading to inefficient timing for faster instructions like ADD.

## Part 2: Multi-Cycle Core

In the second iteration, the processor was refactored into a Multi-Cycle Microarchitecture. This design breaks instruction execution into multiple shorter clock cycles (ranging from 3 to 5 cycles per instruction).

<p align="center">
  <img src="Multi-Cycle%20CPU/Multi_Cycle_CPU_Schematic.svg" alt="Multi Cycle Schematic" width="800">
  <br>
  <em>Figure 2: Complete Multi-Cycle Processor. Adapted from [1].</em>
</p>

### Microarchitecture & FSM
The multi-cycle design employs a **Unified Memory** architecture (Von Neumann) and reuses the ALU for both arithmetic and address calculations, significantly reducing hardware area.

The control logic is driven by a comprehensive **Finite State Machine (FSM)** that orchestrates the datapath:
1. **Fetch:** Load instruction from memory to Instruction Register (IR).
2. **Decode:** Read register file and decode Opcode.
3. **Execute:** Perform ALU operation or calculate branch target.
4. **Memory:** Access memory (Load/Store) if required.
5. **Writeback:** Write result to Register File.

### Key Challenge: Race Conditions
During the implementation of the Multi-Cycle core, a critical **Write-After-Write (WAW) race condition** was identified. The Processor and Memory modules were attempting to drive/read the shared bus on the same active clock edge.
- **Symptom:** Metastable values appearing on the Address Bus during Store operations.
- **Solution:*** The memory write logic was shifted to the **negative clock edge** (`negedge clk`). This ensures that the Address and Data signals from the processor (updated on `posedge`) have fully settled before the memory commits the write.
```
// Fix applied to memory module to prevent race conditions
always_ff @(negedge clk) 
    if (we) RAM[a[31:2]] <= wd;
```



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

<p align="center">
  <img src="Pipelined%20CPU/Pipelined_CPU_Schematic.svg" alt="Pipelined Schematic" width="800">
  <br>
  <em>Figure 2: Complete Pipelined Processor. Adapted from [1].</em>
</p>





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

## References
[1] S. L. Harris and D. M. Harris, *Digital Design and Computer Architecture: RISC-V Edition*. Cambridge, MA: Morgan Kaufmann, 2022. 
