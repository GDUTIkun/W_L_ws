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
- The 2026-08-27 representation audit rejects another single-point replacement. A deterministic canonical-state selector of the lowest eight compiled mesh vertices per wheel, with an independent 3D force at each vertex, represents all 537 original and all 540 fresh valid pre-failure truth resultants at `1e-7` wrench error. The superseding local oracle nevertheless rejects it: rolling set-member switches create a same-pose point jump of `44.93 mm` and a canonical reduced point-Jacobian jump of `0.0403`, so `Jdot_nu` is undefined at the switch. It is no longer an authorized QP candidate.
- The superseding continuous candidate uses the analytic frame `t_r=normalize(a×n)`, `t_l=n×t_r`, a circular contact center, and six permanent virtual surface points: two bottom lateral endpoints plus four 1 mm band-edge corners. It has no vertex membership and passes geometry/Jacobian/bias, physical force-map/virtual-work, original `537/537` and fresh `540/540` resultant gates. Per-point force order is contact-frame `[rolling,lateral,normal]`, transformed to world FLU; left block remains before right.
- Contact kinematics is not copied from the six force points. The validated soft-task candidate is three rows per wheel at the analytic contact center, `A_side=[t_r t_l n]^T J_material N`; velocity FD error is `1.52e-7 m/s` and `Adot_nu` cross error is `2.84e-6 m/s²`. This does not authorize a hard rigid contact constraint.
- The resultant force contract can be condensed exactly at the analytic contact center in the continuous contact frame with order `[F_r,F_l,F_n,M_r,M_l,M_n]`. The six fixed point offsets and `mu=1` pyramids generate 24 unique rays and a fixed 37-row homogeneous H-cone. Point-force, V-ray and H-cone membership agree on all 1,240 cases, including original `537/537` and fresh `540/540` truth resultants. Pose dependence is isolated to the wrench transform/generalized-force map.
- The raw zero-velocity/zero-acceleration coverage result remains `122/173`: 44 failures are outside the Phase 15 componentwise envelope and seven are inside it. The closing audit proves that all seven first fail in the six floating-base rows, violate the same local pitch-moment support facet, are also static-infeasible in a same-configuration full-plant rigid-force oracle, and pass the rolling dynamic-consistency probe where applicable. Phase 15 defines a reconstruction/kinematic workspace, not a static-admissible set; Phase 20 freezes one static equilibrium and an independently generated dynamic envelope. The seven states are therefore classification A (static-gate semantics), not reduced-model defects. The 12D internal-force nullspace per wheel may be eliminated, and the 42D hard-QP is authorized only as the next candidate.
- Positive normal force is world `+Z`; no plant contact force is fed into Controller Core.

## Evidence

The v1 oracle failed the contact-point velocity gate because it mixed a world-vertical geometric offset with a material-point Jacobian. v2 fixed that error. v3/v4 preserved failures in the independent `Jdot_nu` oracle until v5 used a converged midpoint flow. v6/v7 then exposed that the Phase 15 point and compliant Phase 20 passive state do not satisfy the ideal reduced static compatibility condition. v8 froze the current-nominal effective COP and passed static dynamics with `1.89e-14` residual, alongside the closure, tangent, conditioning, passive velocity, mass, actuator, gravity/potential, `Jdot_nu`, Coriolis power, contact finite-difference, wrench-map, virtual-work and forward/inverse gates.

The 12D state/passive reconstruction remains frozen. The continuous six-point force representation, separate three-row soft Pfaffian and exact contact-centered 6D wrench cone pass their local oracles; see [continuous_contact_representation.md](continuous_contact_representation.md), [contact_centered_wrench_condensation.md](contact_centered_wrench_condensation.md), [static_failure_attribution.md](static_failure_attribution.md), and [base_contact_compatibility_closing.md](base_contact_compatibility_closing.md). DG21-01 and DG21-02 are closed and P21-T03 is complete. Historical 36D QP/solver and nonlinear evidence still cannot be inherited; the newly authorized 42D candidate must start at P21-T04.
