#pragma once
#include "BISMOInstruction.hpp"
#include <vector>

//#define DEBUG_EXEC_MODEL

namespace StageModels {
typedef uint64_t BitVector;
typedef int32_t Accumulator;

#define acc(x, y)     acc_ptr[N * x + y]
#define res(x, y, z)  res_ptr[x * N * resmem_size + y * resmem_size + z]

// functional (non-timed) model of an execute stage
template <
  // DPA spatial dimensions
  size_t M, size_t N,
  size_t lmem_size, size_t rmem_size, size_t resmem_size
  // number of buffers in lhs, rhs and result buffers
> void ExecSingleInstr(
  BISMOInstruction in,                  // instruction to execute
  const BitVector lmem[M][lmem_size],   // LHS memory
  const BitVector rmem[N][rmem_size],   // RHS memory
  Accumulator * acc_ptr,                // accumulators [M, N]
  Accumulator * res_ptr                 // result memory [M, N, resmem_size]
) {
  BISMOExecRunInstruction ins = in.exec;
  if(ins.isRunCfg == 0) {
    // sync instructions are simply ignored (assumed to resolve)
    // immediately
    return;
  } else {
    for(size_t m = 0; m < M; m++) {
      for(size_t n = 0; n < N; n++) {
        if(ins.clear_before_first_accumulation == 1) {
          acc(m, n) = 0;
        }
        for(size_t k = 0; k < ins.numTiles; k++) {
          const BitVector opA = lmem[m][ins.lhsOffset + k];
          const BitVector opB = rmem[n][ins.rhsOffset + k];
          const Accumulator contr = __builtin_popcountll(opA & opB);
          const Accumulator wcontr = (contr << ins.shiftAmount);
          const Accumulator wscontr = ins.negate ? -wcontr : contr;
#ifdef DEBUG_EXEC_MODEL
          cout << "Exec Op: m n k = " << m << " " << n << " " << k << " ";
          cout << "opA = " << hex << opA << dec << " ";
          cout << "opB = " << hex << opB << dec << " ";
          cout << "contr = " << contr;
          cout << endl;
#endif
          acc(m, n) +=  wscontr;
        }
        if(ins.writeEn) {
          res(m, n, ins.writeAddr) = acc(m, n);
#ifdef DEBUG_EXEC_MODEL
          cout << "Exec ResWrite: m n addr = " << m << " " << n << " " << ins.writeAddr << " ";
          cout << " <= " << acc(m, n) << endl;
          cout << "ind: " <<  (m * N * resmem_size + n * resmem_size + ins.writeAddr) << endl;
#endif
        }
      }
    }
  }
}

// functional (non-timed) model of an execute stage
// multiple instructions
template <
  // DPA spatial dimensions
  size_t M, size_t N,
  // number of buffers in lhs, rhs and result buffers
  size_t lmem_size, size_t rmem_size, size_t resmem_size
> void ExecMultiInstr(
  std::vector<BISMOInstruction> instrs,  // instruction queue to execute
  const BitVector lmem[M][lmem_size],    // LHS memory
  const BitVector rmem[N][rmem_size],    // RHS memory
  Accumulator * acc_ptr,                 // accumulators [M, N]
  Accumulator * res_ptr                  // result memory [M, N, resmem_size]
) {
  // simply call exec on each instruction
  for(auto & i : instrs) {
    ExecSingleInstr<M, N, lmem_size, rmem_size, resmem_size>(
      i, lmem, rmem, acc_ptr, res_ptr
    );
  }
}

}
