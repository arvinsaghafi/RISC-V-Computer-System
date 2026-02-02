# RISC-V Computer System

![SystemVerilog](https://img.shields.io/badge/Language-SystemVerilog-181717?style=for-the-badge&logo=systemverilog)
![Toolchain](https://img.shields.io/badge/Tools-Quartus%20Prime%20|%20QuestaSim-003366?style=for-the-badge)
![ISA](https://img.shields.io/badge/ISA-RISC--V_RV32I-blue?style=for-the-badge)

A comprehensive implementation of the RISC-V (RV32I) architecture in SystemVerilog. This project explores the evolution of processor microarchitecture by iterating through three distinct designs: **Single-Cycle**, **Multi-Cycle**, and **Pipelined**. The core is designed to be synthesizable for Intel FPGAs and has been verified using a custom assembly stress-test suite.

## Contents
- [Overview](#overview)
- [Repository Structure](#repository-structure)
- [Part 1: Single-Cycle Core](#part-1-single-cycle-core)
  - [Microarchitecture](#microarchitecture-single)
- [Part 2: Multi-Cycle Core](#part-2-multi-cycle-core)
  - [Microarchitecture](#microarchitecture-multi)
  - [Verification: FSM Stress Test](#verification-fsm-stress-test)
  - [Key Challenge: Race Conditions](#key-challenge-race-conditions)
- [Part 3: Pipelined Core](#part-3-pipelined-core)
  - [Microarchitecture](#microarchitecture-pipelined)
  - [Hazard Handling](#hazard-handling)
  - [Verification: Hazard Unit Stress Test](#verification-hazard-unit-stress-test)
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

## Repository Structure

To ensure reproducibility, this repository includes source code, Quartus project files, and simulation testbenches.
```
RISC-V-Computer-System/
├── Single-Cycle CPU/         # Iteration 1 Source Code
│   ├── top.sv                # Top-level design for Single-Cycle
|   ├── imem.txt              # Instruction Memory
│   └── dmem.txt              # Data Memory Initialization
├── Multi-Cycle CPU/          # Iteration 2 Source Code
|   ├── top.sv                # Top-level design for Multi-Cycle
│   ├── tb_top.sv             # Top-level verification for Multi-Cycle
│   └── mem.txt               # Hex machine code for FSM stress test
├── Pipelined CPU/            # Iteration 3 Source Code
|   ├── top.sv                # Top-level design for Pipelined
│   ├── tb_top.sv             # Top-level verification for Pipelined
│   ├── imem.txt              # Instruction Memory (Hazard Test Suite)
│   └── dmem.txt              # Data Memory Initialization
└── README.md
```

## Part 1: Single Cycle Core

The first iteration implements the entire instruction execution in a single clock cycle. While conceptually simple, the clock period is limited by the longest path in the circuit (Load Word instruction).

<p align="center">
  <img src="Single-Cycle%20CPU/Single_Cycle_CPU_Schematic.svg" alt="Single Cycle Schematic" width="800">
  <br>
  <em>Figure 1: Complete Single-Cycle Processor. Adapted from [1].</em>
</p>

### Microarchitecture <a name="microarchitecture-single"></a>

The single-cycle design utilizes a Harvard Architecture, maintaining separate physical memories for Instructions (`imem`) and Data (`dmem`). This allows the processor to fetch an instruction and access data memory in the same clock cycle without contention.

- **Control Unit:** A dedicated main decoder and ALU decoder generate control signals combinatorially based on the Opcode and Funct3/7 fields.
- **Memory Mapped I/O:** The top-level module maps specific memory addresses to FPGA peripherals, allowing software to control hardware directly:
  - `0xFF200000`: LEDR (Red LEDs)
  - `0xFF200020`: HEX3-HEX0 (7-Segment Displays)
  - `0xFF200030`: HEX5-HEX4
- **CPI:** 1 (Theoretical)
- **Limitation:** The clock frequency is severely limited by the critical path delay ($T_{clk} \geq T_{fetch} + T_{decode} + T_{execute} + T_{memory} + T_{writeback}$).

## Part 2: Multi-Cycle Core

In the second iteration, the processor was refactored into a Multi-Cycle Microarchitecture. This design breaks instruction execution into multiple shorter clock cycles (ranging from 3 to 5 cycles per instruction).

<p align="center">
  <img src="Multi-Cycle%20CPU/Multi_Cycle_CPU_Schematic.svg" alt="Multi Cycle Schematic" width="800">
  <br>
  <em>Figure 2: Complete Multi-Cycle Processor. Adapted from [1].</em>
</p>

### Microarchitecture <a name="microarchitecture-multi"></a>
The multi-cycle design employs a Unified Memory architecture (Von Neumann) and reuses the ALU for both arithmetic and address calculations, significantly reducing hardware area. The control logic is driven by a comprehensive Finite State Machine (FSM) that orchestrates the datapath:

1. **Fetch:** Load instruction from memory to Instruction Register (IR).
2. **Decode:** Read register file and decode Opcode.
3. **Execute:** Perform ALU operation or calculate branch target.
4. **Memory:** Access memory (Load/Store) if required.
5. **Writeback:** Write result to Register File.

### Verification: FSM Stress Test
To validate the FSM transitions, a stress test (mem.txt) was developed to exercise Arithmetic, Branching, and Memory operations in a single continuous run.

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

**Simulation Results:** The testbench (tb_top.sv) monitors the memory bus. Success is confirmed when the value `25` (0x19) is written to address `100` (0x64), confirming all logic blocks are functional.

<img width="2447" height="706" alt="riscvmulti_testbench_questa_wave" src="https://github.com/user-attachments/assets/d95b4782-18fa-4bf9-ba6a-de1c13f5bf58" />

*Figure 3: Final waveform verification showing the "Magic Number" 0x19 being written to address 0x64 at the end of the simulation.*

### Key Challenge: Race Conditions
During the implementation of the Multi-Cycle core, a critical **Write-After-Write (WAW) race condition** was identified. The Processor and Memory modules were attempting to drive/read the shared bus on the same active clock edge.
- **Symptom:** Metastable values (`Address = 63`) appearing on the Address Bus during Store operations.
- **Solution:** The memory write logic was shifted to the **negative clock edge** (`negedge clk`). This ensures that the Address and Data signals from the processor (updated on `posedge`) have fully settled before the memory commits the write.
```
// Fix applied to memory module to prevent race conditions
always_ff @(negedge clk) 
    if (we) RAM[a[31:2]] <= wd;
```

## Part 3: Pipelined Core

The final iteration implements a 5-stage pipeline to maximize throughput by exploiting Instruction Level Parallelism (ILP).

<p align="center">
  <img src="Pipelined%20CPU/Pipelined_CPU_Schematic.svg" alt="Pipelined Schematic" width="800">
  <br>
  <em>Figure 4: Complete Pipelined Processor. Adapted from [1].</em>
</p>

### Microarchitecture <a name="microarchitecture-pipelined"></a>
The pipelined core divides the instruction execution into five discrete stages, separated by pipeline registers (IF/ID, ID/EX, EX/MEM, MEM/WB). This allows up to five instructions to be active simultaneously.

1. **Fetch (IF):** Reads instruction from Instruction Memory.
2. **Decode (ID):** Decodes instruction and reads Register File.
3. **Execute (EX):** Performs ALU calculations.
4. **Memory (MEM):** Reads/Writes Data Memory.
5. **Writeback (WB):** Writes results back to Register File.

### Hazard Handling
To maintain data integrity without stalling the pipeline unnecessarily, a dedicated Hazard Unit was implemented:
1. **Data Hazards (RAW):** Solved via Forwarding (bypassing the Register File to feed ALU results directly back to EX stage).
2. **Load-Use Hazards:** Solved via Stalling (freezing the PC and IF/ID registers for 1 cycle).
3. **Control Hazards (Branching):** Solved via Flushing (clearing the IF/ID and ID/EX registers when a branch is taken).

### Verification: Hazard Unit Stress Test
A specific test suite (`imem.txt`) was written to deliberately induce hazards and verify the CPU's automatic resolution. The testbench (`tb_top.sv`) peeks into the register file to verify results.

**Test Case 1: Data Hazard & Forwarding**
```
add  x3, x1, x2   ; x3 = 15 (Result in EX stage)
sub  x4, x3, x1   ; x4 = x3 - x1. DEPENDENCY on x3!
// The Hazard Unit must forward x3 from EX/MEM pipeline register to ALU.
```
*Result:* `x4` correctly resolved to `10`.

**Test Case 2: Load-Use Stall**
```
lw   x5, 0(x0)    ; Load 99
add  x6, x5, x5   ; Use 99 immediately.
// The Hazard Unit must stall the pipeline for 1 cycle.
```
*Result:* `x6` correctly resolved to `198`.

**Test Case 3: Control Hazard Flush**
```
beq  x7, x7, 6    ; Branch Taken.
addi x8, x0, 1    ; Should be flushed (Turned to NOP).
addi x8, x0, 2    ; Should be flushed (Turned to NOP).
addi x8, x0, 3    ; Target.
```
*Result:* `x8` remains `3`, confirming intermediate instructions were flushed.

**Simulation Results:** The waveform below demonstrates the StallF and FlushE signals firing correctly during the execution of the stress test.

<img width="2385" height="964" alt="riscvpipelined_testbench_questa_wave" src="https://github.com/user-attachments/assets/a35a4c59-cc74-4a4e-a920-2db35875f5d1" />

*Figure 5: Pipelined simulation waveform captured in QuestaSim, showing the orchestration of pipeline control signals.*

## Usage & Simulation
To reproduce the verification results using ModelSim or QuestaSim:

**1. Clone the repository:**
```
git clone https://github.com/arvinsaghafi/RISC-V-Computer-System.git
cd riscv-computer-system
```
**2. Compile and Run the Multi-Cycle Simulation:**
```
vlib work
vlog Multi-Cycle\ CPU/testbench.sv Multi-Cycle\ CPU/top.sv
vsim -voptargs=+acc work.testbench
run -all
```

*Expected Output:* Simulation Succeeded! Value 25 written to address 100.

**3. Compile and Run the Pipelined Simulation:**
```
vlib work
vlog Pipelined\ CPU/tb_top.sv Pipelined\ CPU/top.sv
vsim -voptargs=+acc work.tb_top
run -all
```
*Expected Output:*
```
[PASS] Basic ALU:  x3 = 15
[PASS] Forwarding: x4 = 10
[PASS] Load Word:  x5 = 99
[PASS] Stall/LU:   x6 = 198
[PASS] Branching:  x8 = 3
```

## References
[1] S. L. Harris and D. M. Harris, *Digital Design and Computer Architecture: RISC-V Edition*. Cambridge, MA: Morgan Kaufmann, 2022.

## Contact

If you have any questions regarding the implementation, verification methodology, or synthesis, feel free to reach out to me:

**Arvin Saghafi** <br>
*Electrical Engineering Student* <br>
LinkedIn: https://www.linkedin.com/in/arvinsaghafi/
