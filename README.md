# Pipelined MIPS-like Processor with Cache and Memory Modules

This project implements a 5-stage pipelined processor using Verilog, closely resembling a basic MIPS architecture. It integrates a cache memory, main memory, and register file, providing a full demonstration of pipelined execution along with memory hierarchy interaction.
Architecture Overview

The processor follows the standard 5-stage pipeline:

    FETCH → DECODE → EXECUTE → MEMORY → WRITE BACK

Each pipeline stage is separated by dedicated pipeline registers to maintain smooth instruction flow.
## Major Components
| Module            | Description                                                                                                                              |
| ----------------- | ---------------------------------------------------------------------------------------------------------------------------------------  | 
| instructionMemory | Fetches 32-bit instructions from a ROM (program_cacheTest.mem).                                                                          |
| registerFile      | 32 x 8-bit register file with read/write support. Auto-dumps to reg_dump.mem.                                                            |
| decode            | Decodes R-type, I-type, load (lw), and store (sw) instructions. Generates ALU control signals.                                           |
| ALU               | 	8-bit ALU supporting operations like ADD, SUB, AND, OR, XOR, SLL, SRL, and SLT.                                                        |
| storage           | 256 x 8-bit main memory with read/write and intentional delays to simulate memory access times. Dumps to memory_dump.mem.                |
| cacheMemory       | 4-block, 4-byte, direct-mapped cache with valid, dirty, and tag bits. Interfaces with storage for cache misses. Dumps to cache_dump.mem. |
| ifid_register     | Pipeline register between Instruction Fetch and Decode stages.                                                                           |
| idex_register     | Pipeline register between Decode and Execute stages.                                                                                     |
| exmem_register    | Pipeline register between Execute and Memory stages.                                                                                     |
| memwb_register    | Pipeline register between Memory and Write Back stages.                                                                                  |
| aluSrcMuxer       | Selects between register data and immediate values for ALU operations.                                                                   |
| wbDataMux         | Selects between memory read data and ALU result for register write-back.                                                                 |

## Supported Instruction Types

    R-Type: Register-Register ALU operations

    I-Type: Immediate ALU operations

    LW: Load Word

    SW: Store Word

## Special Features

    Cache and Memory Integration: Handles cache hits, cache misses (with dirty write-backs), and memory stalls.

    Memory Access Stalls: Pipeline can stall during cache misses and memory access delays.

    Full Pipeline Flow: All pipeline registers and control hazard handling are modeled.

    Data Dumping: Register, memory, and cache contents are periodically written to .mem files for debugging.

## Files Used
|  File                      |	Purpose                                      |
|  ------------------------- |  -------------------------------------------- |
|  program_cacheTest.mem	   |  Instruction set ROM file.                    |
|  register.mem              |  Initial register file content.               |
|  memoryData.mem            |  Main memory content.                         |
|  reg_dump.mem              |  Dump of register contents after operations.  |
|  memory_dump.mem           |  Dump of memory contents after operations.    |
|  cache_dump.mem            |  Dump of cache contents after operations.     |

### Notes

    Memory and cache accesses are intentionally slowed down to mimic realistic memory latencies.

    System assumes 8-bit data width for registers and memory, 32-bit for instructions.
