# Phase 21 Reduced Model Contract

State/reconstruction authority: `current_nominal_weighted_wbc_model_oracle_v2`  
Dynamics/contact authority: **reopened by nonlinear pre-freeze**  
Latest local oracle: `current_nominal_weighted_wbc_model_oracle_v9`  
Latest result: `data/experiments/2026-08-26-phase21-model-oracle-v9/summary.json`

## Coordinates

Configuration is `(p_B^N, q_N_from_B, q_active_canonical)` with the canonical active order
`[left_hip,left_knee,left_wheel,right_hip,right_knee,right_wheel]`.

The 12D tangent and QP acceleration order is:

`[v_Bx^N,v_By^N,v_Bz^N,omega_Bx^N,omega_By^N,omega_Bz^N,dq_active_canonical_6]`.

The mapping from this tangent to MuJoCo's 16D tree velocity is recomputed from the world-axis base-control-site twist Jacobian and the Phase 15 closure tangent. Canonical active joint velocity and torque signs are the negative of native MuJoCo signs, matching the Adapter.

## Passive Reconstruction

- Solve the two three-row site-closure residuals for four passive joints on the Phase 15 nominal assembly branch.
- Reject non-finite input, non-convergence, closure residual above `1e-10 m`, passive minimum singular value below `0.005`, condition number above `40`, or a state outside the Phase 15 frozen workspace.
- Passive velocity is the analytic closure-tangent reduction. It is not added to public `RobotState`.

## Contact

- Phase 15 local `[0.05,0,0] m` remains the kinematic seed, but it cannot reproduce the Phase 20 mesh contact equilibrium as a single-force WBC model.
- v8 uses each wheel's Phase 20 force-weighted mesh-contact COP, expressed in the wheel-body frame, plus a shared local-X `-18.68650895129817 um` correction for ideal-closure static compatibility. It closes the equilibrium equations but is a fixed material point and does not follow the rolling mesh contact.
- v9 tested an instantaneous selector that keeps the equilibrium-calibrated offset in world axes and reselects it from the wheel center. It passes the local analytic/finite-difference gates but does not pass the nonlinear plant gate either. Neither v8 nor v9 is therefore production authority.
- Force order is left `[rolling world +X,lateral world +Y,normal world +Z]`, then right in the same order.
- Positive normal force is world `+Z`; no plant contact force is fed into Controller Core.

## Evidence

The v1 oracle failed the contact-point velocity gate because it mixed a world-vertical geometric offset with a material-point Jacobian. v2 fixed that error. v3/v4 preserved failures in the independent `Jdot_nu` oracle until v5 used a converged midpoint flow. v6/v7 then exposed that the Phase 15 point and compliant Phase 20 passive state do not satisfy the ideal reduced static compatibility condition. v8 froze the current-nominal effective COP and passed static dynamics with `1.89e-14` residual, alongside the closure, tangent, conditioning, passive velocity, mass, actuator, gravity/potential, `Jdot_nu`, Coriolis power, contact finite-difference, wrench-map, virtual-work and forward/inverse gates.

The 12D state/passive reconstruction remains frozen. The contact portion is reopened: v8 and v9 show that a single equilibrium-calibrated force point can pass local/static gates while failing the 10 s rolling plant. P21-T03/DG21-01/02 cannot close again until a state-dependent rolling-contact representation passes both the independent oracle and nonlinear holdout without plant-private feedback.
