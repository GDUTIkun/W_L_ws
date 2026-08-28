# P21-T08 Runtime Core Integration

Date: 2026-08-28
Base revision: `deb15da3aa41411ce53d5bae94fd02fb85107c97`

## Result

P21-T08 is **PASS**. The production Core now has an additive, opt-in
`ControllerMode::kWeightedWbc` path. It preserves the canonical
`RobotState -> TorqueCommand` boundary, does not introduce a MuJoCo
dependency, and leaves `kZero`, joint PD/gravity, planar standing, and 3D
standing behavior in their existing branches.

The nominal producer is frozen to the P21-T06 equilibrium, gains, 12D
interaction wrench, world-axis shortest-arc orientation convention, 10 ms
control period, and reset-time x/y/heading anchors. The Core passes the
generated `WbcReference` to the P21-T07 `WeightedWbcController` without
changing its model, 42D problem, solver, sign/order, or hard-bound contracts.

## Safety contract

- Invalid or non-monotonic canonical state produces six zero torques and
  latches the weighted mode.
- Contact loss, a control-period mismatch greater than 1 us, or an x/y/z/
  roll/pitch/yaw envelope violation produces six zero torques and latches.
- Model, problem, solver, hard-residual, or non-finite failure produces six
  zero torques and latches.
- A candidate exceeding the configured Core torque limit is rejected without
  clipping or reuse of the previous command.
- The source sample timestamp is retained on accepted and rejected ticks.
- `reset()` clears the safety latch, reference anchors, timestamp history,
  and solver warm start.

## Attribution test

`test_controller_core_weighted_wbc` reads the first case from the existing
P21-T07 32-case golden corpus. At the exact equilibrium:

- the Core-generated reference matches all 21 golden reference components
  within `1e-12`;
- the six Core torques match an independent cold
  `WeightedWbcController::step` within `1e-12 N.m`;
- controller/model/solver status, iterations, hard violation, and stationarity
  diagnostics match the independent result;
- cold, warm 10 ms, and reset-cold behavior is deterministic.

The same test separately covers bilateral-contact loss, 20 ms timing, x/y/z,
roll/pitch/yaw envelope violations, invalid and non-monotonic state, a
positive-but-too-small output limit, latch persistence/reset recovery, and a
`kZero` compatibility check.

## Commands and observed results

Run from `ros_ws/`:

```text
colcon build --packages-select wheel_leg_core --cmake-args -DCMAKE_BUILD_TYPE=Release
ctest --test-dir build/wheel_leg_core --output-on-failure
```

Observed result: Release build PASS; CTest `6/6` PASS, zero failures. The
new Core integration test completed in approximately 0.15 s in the independent
verification run. `git diff --check` also passed.

A bounded literal scan of `wheel_leg_core/include`, `wheel_leg_core/src`,
and its `CMakeLists.txt` found no MuJoCo symbol or include.

## Input hashes

```text
22ea0d9ccd19363d8cd3bb552361c18884ba0193608d2386831d1e3d4991c046  include/wheel_leg_core/controller_core.hpp
18fe8ca89f1a16750eb4b8c3d8fbafc75372c1e36514a53e941a4a6f2023058b  src/controller_core.cpp
4b0eb515a4bf7f055fef31e9f27aa5190aad0f1c918bd95d3b27d1d0232d7750  test/test_controller_core_weighted_wbc.cpp
7b70df0573cfa1e0be2619677ca2ba8a432fccdb0141255889acdd3c957dedf1  CMakeLists.txt
0c49680ff833be3b771b8eb51d66e124f268e295953d307a9168e05359e29a39  test/data/phase21_weighted_wbc_problem_golden_v2.txt
```

## Scope boundary

This evidence closes the Core portion of DG21-06 only. It is not MuJoCo
closed-loop evidence and does not close the 5-step ZOH, dual-clock, runner,
formal, replay, or reuse gates. Those remain P21-T09 through P21-T11.
