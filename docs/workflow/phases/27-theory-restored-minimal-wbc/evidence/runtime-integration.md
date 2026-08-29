# Phase 27 Runtime Integration Evidence

Date: 2026-08-29

Decision: `T08 PASS`

## Runtime contract

- The new controller is opt-in as `kPhase27MinimalNmpcWbc`; the production
  default and Phase 23 `kNominalNmpcWbc` mode are unchanged.
- MuJoCo remains at 2 ms, Controller/WBC at 10 ms and the 16-state NMPC at
  every second control tick (20 ms). The held wrench has age 0 then 1; age 2
  is stale and latches the controller.
- Planner state, yaw-aligned reference state, NMPC warm start and Minimal WBC
  warm start are reset together. Solver failure, late completion, stale data
  and non-finite output command six exact zeros and latch until reset.
- Control logs add planner, NMPC audit/timing, requested/realized interaction
  wrench, residual and signed-slack columns. Existing columns and plant logs
  are not replaced. The runner refuses to overwrite either output.

## Verification

The Release component suite passed 32/32 tests. In particular,
`wheel_leg_core_controller_core_phase27` checks update/hold ages, deterministic
reset replay, all four NMPC fault classes, exact-zero latch/reset and rejected
out-of-contract references. Existing Core, ROS and MuJoCo tests remain part of
the full regression command.

A headless `phase19_standing.xml` smoke run produced 100 control ticks with the
new additive schema. It intentionally is not formal evidence: the Minimal
candidate crossed the existing x safety envelope at tick 58, which is a
controller-level behavior to be judged by the predeclared T0 gate in T10, not
an integration failure or permission to add a hidden state task.

Separate three-tick runner probes injected `nmpc_solver_failure`, `nmpc_late`,
`nmpc_stale` and `nmpc_nonfinite` at tick 0. Every case logged status 4,
latch 1 and six exact-zero command torques. Reusing an existing control path
returned exit code 1 with `Refusing to overwrite output`.

This closes runtime wiring and observability only. It does not close DG27-05,
any T0--T3 behavior gate, or the final Minimal PASS/FAIL decision.
