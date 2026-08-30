# Phase 41 regression audit

DG41-01: **PASS**.

Targeted ROS build completed for `wheel_leg_core` and `wheel_leg_mujoco`. Targeted test result:
35 tests, 0 errors, 0 failures, 0 skipped.

Explicit contract assertions establish:

- left wheel q shifted by 2π: workspace inside and model OK;
- right wheel additionally shifted by 10π: workspace inside and model OK;
- wheel q = Inf: `kInvalidState`;
- all four hip/knee entries accept exact bounds and reject 1e-9 rad beyond either bound;
- a leg violation still produces `kOutsideWorkspace` and WBC problem rejection.

Existing test authorities remain green for RobotState q/dq finite validation, Adapter contact
aggregation/watchdogs, ControllerCore contact/safety latch, DenseQp solver rejection, WBC model/
problem/controller hard constraints, torque/slack behavior and Phase27 Minimal. No threshold or test
expectation outside wheel absolute workspace semantics was relaxed.
