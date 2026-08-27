# Phase 21 P21-T01 Grounding

Date: 2026-08-26  
Evidence tier: CBM Verify (Tier 2) plus live-source fallback  
CBM project/generation: `W_L_ws` / `2026-08-26T07:29:33Z` (`full`)  
Source HEAD at grounding: `694963451336e5103afc198d157c155458b78394`

## Live Production Boundary

- `wheel_leg_core/types.hpp` freezes public `RobotState` and `TorqueCommand`; Phase 21 does not add MuJoCo state, passive joints, mass matrices or contact wrench to that boundary.
- `ControllerCore::step` in `controller_core.cpp` is the single Core control entry. The live modes are zero, Joint-PD/gravity, planar simple standing and full-3D simple standing; no production WBC/QP exists.
- `Adapter::extractState`, `acceptCommand` and `writeControls` own MuJoCo-to-canonical state/sign/contact conversion and timestamp/watchdog/fail-zero behavior. Phase 21 reuses them unchanged.
- `standing_3d_loop.cpp` owns the reusable full-3D `nq=17/nv=16/nu=6`, 2 ms physics, 10 ms control and five-step ZOH runner pattern. It is not a WBC implementation.

## Reuse

- Phase 14/18: current nominal plant, floating-base/contact validation entry points and contact truth logging.
- Phase 15 `run_mujoco_closed_chain_kinematics.py`: passive branch solve, closure Jacobian reduction, finite-difference, velocity and virtual-work oracles.
- Phase 20: equilibrium initial state, nonlinear case envelope, independent runner/evaluator/manifest/replay/non-overwrite structure.
- Simulink `spatial_two_leg_qp_core.m` and `controller_qp_core.m`: semantic comparison for mass/contact/wrench/task equations and test cases only.

## Must Not Be Copied as Authority

- Phase 20's static identified/LQR gain and simple-standing torque decomposition do not establish WBC dynamics, QP feasibility or task correctness.
- Simulink `evalin`, variable-width parsing, 5 ms timing, `quadprog`, `pinv` fallback, persistent warm start, numerical weights and legacy contact row selection do not enter production unchanged.
- Historical simulation PASS does not close Phase 21 model, solver, task, deadline, formal or real-hardware gates.

## Coverage and Fallback

- `controller_core.hpp/.cpp` were reported `metadata_changed`; both were read directly.
- `adapter.hpp:28` and `deterministic_loop.cpp:438` were reported partial; the indicated ranges and surrounding source were read directly.
- `standing_3d_loop.cpp` was reported not tracked by the recorded generation and was read directly.
- Other cited candidate paths had no recorded coverage issue. This is a best-effort signal, not proof of completeness.

## Frozen First Implementation Surface

1. Define the 12D canonical tangent as world-axis base linear/angular velocity followed by six canonical active-joint velocities.
2. Reconstruct the Phase 15 passive branch and its velocity reduction; fail closed outside the frozen workspace or conditioning thresholds.
3. Use MuJoCo only as the independent offline plant oracle for `M_r/h_r/S_r/J_c/Jdot_nu` and sign/order tests. A production model must later be runtime-independent.
4. Audit and freeze exactly one convex inequality-QP solver before Core integration; equality-only KKT or post-solve clipping cannot be production fallback.
5. Limit the first production surface to an additive WBC module/mode and independent full-3D runner. Existing modes, public messages and hardware paths remain unchanged.

