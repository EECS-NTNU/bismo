# Instrumentation

Performance is a key consideration for BISMO, and the only way to understand
performance is to measure it. Towards this end, the BISMO hardware and runtime
library provide support for collecting instrumentation data during execution.
This is helpful for understanding and fixing performance problems.

## How do I get these?

* **Using the BISMO top-level test application:** Run the BISMO top-level test app with `i` for  interactive benchmarking and `b` for batch-mode benchmarking, e.g. `.\testapp i`. Enter `<lhsrows> <lhscols> <rhscols> <lhsbits> <rhsbits>` from `stdin` in either mode to launch a matrix
multiplication of the specified size and get the metrics printed in `stdout`.

* **In your own code or application:** Call the `getInstrumentationData()` function after
having executed an `execMatMul()` and read out desired metrics from the returned
`InstrumentationData` instance.

## Metrics

The following metrics are currently available:

| Metric Name  | Description       | Unit |
| ------------- |:-------------:|-------------:|
| actual_read_oi | "Actual" read OI, taking into account the re-reads from DRAM due to tiling strategy | binops/byte |
| actual_write_oi | "Actual" write OI, since we write only once should be the same as workload_write_oi | binops/byte |
| hw_buf_size_bytes | Total size of the hardware matrix OCM buffers | byte |
| hw_fclk_mhz | Measured hardware clock | MHz |
| hw_peak_perf_binops | Theoretical peak performance of overlay | Gbinops/sec |
| hw_peak_read_bw | Theoretical peak DRAM read bandwidth per cycle | bytes/cycle |
| hw_peak_read_oi | Read OI needed to reach HW peak performance | binops/byte |
| hw_peak_write_bw | Theoretical peak DRAM write bandwidth per cycle | bytes/cycle |
| hw_peak_write_oi | Write OI needed to reach HW peak performance | binops/byte |
| mat_lhs_host2accel_us | Host to accel data transfer time for LHS | microseconds |
| mat_lhs_p2s_us | Time spent on parallel-to-serial for LHS | microseconds |
| mat_lhs_pad_us | Time spent on padding for LHS | microseconds |
| mat_res_accel2host_us | Accel to host data transfer time for Res | microseconds |
| mat_res_unpad_us | Time spent on removing padding for Res | microseconds |
| mat_rhs_host2accel_us | Host to accel data transfer time for RHS | microseconds |
| mat_rhs_p2s_us | Time spent on parallel-to-serial for RHS | microseconds |
| mat_rhs_pad_us | Time spent on padding for RHS | microseconds |
| run_achieved_binops | Achieved performance excluding p2s and host<->accel | Gbinops/sec |
| run_cycles | Number of cycles taken excluding p2s and host<->accel | cycles |
| run_eff%_exec | Efficiency for execute stage | percent |
| run_eff%_fetch | Efficiency for fetch stage | percent |
| run_eff%_result | Efficiency for result stage | percent |
| stg_exec_idle | Cycles spent idle for execute stage | cycles |
| stg_exec_rcv | Cycles spent waiting for tokens for execute stage | cycles |
| stg_exec_run | Cycles spent running for execute stage | cycles |
| stg_exec_snd | Cycles spent sending tokens for execute stage | cycles |
| stg_fetch_idle | Cycles spent idle for fetch stage | cycles |
| stg_fetch_rcv | Cycles spent waiting for tokens for fetch stage | cycles |
| stg_fetch_run | Cycles spent running for fetch stage | cycles |
| stg_fetch_snd | Cycles spent sending tokens for fetch stage | cycles |
| stg_result_idle | Cycles spent idle for result stage | cycles |
| stg_result_rcv | Cycles spent waiting for tokens for result stage | cycles |
| stg_result_run | Cycles spent running for result stage | cycles |
| stg_result_snd | Cycles spent sending tokens for result stage | cycles |
| workload_actual_binops | Actual (excluding padding) binops for workload | binops |
| workload_dram_read_bytes | Number of bytes to read from DRAM, including re-reads | bytes |
| workload_dram_write_bytes | Number of bytes to write to DRAM | bytes |
| workload_lhs_bytes | Size of LHS matrix, excluding re-reads | bytes |
| workload_read_oi | Workload read OI, if the entire workload could be read only once and kept on-chip | binops/byte |
| workload_res_bytes | Size of Res matrix | bytes |
| workload_rhs_bytes | Size of RHS matrix, excluding re-reads | bytes |
| workload_total_binops | Total (including padding) number of binops for workload | binops |
| workload_write_oi | Workload write OI | binops/byte |


* OI stands for Operational Intensity, number of total bit operations divided by number of bytes read from or written to DRAM.
* LHS is the left-hand-side matrix, RHS is the right-hand-side matrix, Res is the result matrix
* A binary multiply-accumulate counts as two binops. A `w`-by-`a` bit multiply-accumulate counts as `2 * w * a` binops.

## Understanding stage efficiency
We define the *efficiency* of BISMO stages in a specific way: the number of
theoretical minimum cycles the stage would require divided by the actual number
of cycles spent in the running state reported by the stage controller.

* For the execute stage, the theoretical minimum is determined by the number
of binops required by the workload divided by the theoretical peak binops per cycle
(determined by the overlay/DPA size)
* For the fetch and result stages, the theoretical minimum is determined by
the number of bytes to read or write divided by the DRAM theoretical peak.

*Note that idle cycles (due to e.g. no available instructions) and synchronization
cycles (cycles spent sending or waiting for tokens) are not counted as part of
stage efficiency.*

## How is this implemented?
Instrumentation is partially done using timers on the CPU and partially by
hardware cycle counters.
See the `perfSummary` and `perfDetails` functions in `src/main/resources/lib/bismo_rt_matmul.cpp`.
