# BISMO Instructions

It is possible to bypass the instruction generations and descriptors, feeding instructions into BISMO directly. This is a highly advanced usage mode and we do not recommend this for beginner users.
Structs that have all the instruction encodings, bitfield lengths etc. can be found in:
`src/main/resources/cpp/lib/BISMOInstruction.hpp`

Two kinds of instructions: synchronize and run. Each instruction has `targetStage` and `isRunCfg` fields. 
`targetStage` is the target stage for this instruction (0 - fetch, 1 - execute, 2 - result).
`isRunCfg` determines whether this instruction is a sync or run instruction (0 - sync, 1 - run). 

## Synchronization instructions
Sync instruction are the same for each stage.
`isSendToken` determines whether this is a send token or receive token transaction (0 - receive, 1 - send). 
`chanID` indicates which stage to communicate with, always 0 for fetch and result stages. This is only relevant for the execute stage: (0 - sync with fetch, 1 - sync with result).

## Fetch run instruction
  * unused0 -- set to zero
  * bram_id_start -- 0 for LHS, Dm for RHS
  * bram_id_range -- 0 for LHS, 1 for RHS
  * bram_addr_base -- base address to start writing into each matrix buffer
  * tiles_per_row -- how many writes to do into a particular matrix buffer before moving into the next one
  * dram_base -- physical DRAM address to start reading from
  * dram_block_size_bytes -- number of bytes in one block (to be read contiguously)
  * dram_block_offset_bytes -- stride to get to next block
  * dram_block_count -- number of blocks to do (e.g. precision in current fetch schedule)
  
## Execute run instruction
  * unused0 -- set to zero
  * lhsOffset -- base offset for lhs memory
  * rhsOffset -- base offset for rhs memory
  * numTiles -- number of memory elements to go through for execution
  * shiftAmount -- 0 -- do not shift accumulator before adding, 1 -- shift accumulator before adding
  * negate -- 0 -- do no negate current contribution, 1 -- negate current contribution
  * clear_before_first_accumulation -- 0 -- do not clear accumulator before adding, 1 -- clear accumulator before adding
  * writeEn -- do a write to the result buffer when set
  * writeAddr -- indicates which result buffer to write into, 0 or 1

## Result run instruction
* unused0 (set to zero)
* nop -- just a no-operation result instruction
* resmem_addr -- which on-chip result buffer to read from, there are only two buffers (0 and 1). 
* dram_base -- base physical address to store current result into.
* dram_skip -- the stride between consecutive columns of the result tile.
* waitCompleteBytes -- 0 -- do not wait for all bytes written to return or 1 -- block execution until all bytes written to DRAM
