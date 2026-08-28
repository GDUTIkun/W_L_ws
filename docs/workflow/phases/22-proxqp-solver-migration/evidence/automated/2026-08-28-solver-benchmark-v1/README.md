# Phase 22 production-adapter solver benchmark v1

The production `DenseQpSolver` adapter was built in Release mode and evaluated
against the unchanged Phase 21 32-case corpus for 1000 cold, 1000 repeated-same
warm and 1000 cycling-dynamic warm setup+solve operations.

Result: PASS. Maximum cold/dynamic times were 0.929801/0.689053 ms; maximum
independent stationarity was 2.9134814e-9; maximum bound/equality violations
were 9.7655862e-9; maximum physical-torque oracle difference was
2.8314025e-5 N m; maximum objective gap was 2.4415758e-9. All are inside the
frozen Phase 22 gates.

Command:

```bash
cd /home/t/W_L_ws
ros_ws/build/wheel_leg_core/benchmark_dense_qp_solver \
  data/experiments/2026-08-28-phase21-weighted-solver-runtime-v3/problem_corpus.txt \
  docs/workflow/phases/22-proxqp-solver-migration/evidence/automated/2026-08-28-solver-benchmark-v1/benchmark.json \
  1000
```

The result is reference-host simulation infrastructure evidence only. It does
not establish target-hardware real-time behavior.
