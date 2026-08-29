# Phase 31: Wheel-State Model and Measurement Contract Audit — REVIEW

Status: `review`

Verdict: `REWORK`

Supersession note (Phase32): measurement/kinematics findings remain valid. The controlled dynamics
replay left the XML `base_weld` active and is not authority for the floating production plant;
Phase32 same-x16 floating-base evidence supersedes the M4-only attribution with `P32-C/M5`.

## Findings

1. The wheel-state measurement contract is correct on the authority corpus. Core `xi/dxi`, direct
   MuJoCo geometry, analytic moving-frame velocity, and centered finite differences pass. No Adapter
   or RobotState change is authorized.
2. Eq.(12) is not adequate for the current articulated/contact plant under Minimal WBC. Signs and
   WBC realization pass, but controlled plant gains differ by up to `93.40%`.
3. A scalar `D_eff` cannot repair the model: channel-derived values contradict one another by
   `2.226x`, and common `Fx` has no positive scalar solution.
4. Root cause closes as `P31-E_missing_wheel_kinematic_dynamic_coupling` / `M4`. The evidence does
   not prove that the coordinate itself is unsuitable (`M5`).
5. Soft-contact WBC projection and a simple bilateral-contact KKT both fail independent MuJoCo
   sensitivity gates. Their production-struct changes were removed; only offline tools/evidence and
   the experiment target remain.
6. The restored production tree builds in Release and all `wheel_leg_core` CTest entries pass
   (`15/15`); aggregate `colcon test-result` reports `33 tests, 0 errors, 0 failures, 0 skipped`.

## Verdict

`REWORK — P31-E_missing_wheel_kinematic_dynamic_coupling`

Do not change cost/reference/feedforward/horizon/SQP-RTI, fit a scalar correction, generate a new
acados artifact, or switch production. A new physically derived closed-chain rolling/contact response
model must pass independent sensitivity and 20 ms gates before Phase31 can resume T11–T14. No
`RECORD.md` is created.
