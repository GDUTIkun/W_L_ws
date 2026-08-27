# Phase 21 Failure Attribution Audit

Date: 2026-08-27  
Verdict: **REWORK — contact/model mismatch is the supported upstream blocker; the baseline tick-272 solver failure is numerical, not mathematical infeasibility**

## Frozen audit

This audit did not change task weights, task gains, bounds, solver settings, maximum iterations, timing, plant, model v8, or acceptance gates. It used task-prefreeze v5 (`rho=10`, 4000 iterations, wrench-slack penalty `1`), 2 ms physics, 10 ms control, five-step ZOH, the nominal reset, and 1000 control ticks per case.

Fresh results:

- attribution matrix: `data/experiments/2026-08-27-phase21-attribution-v2/`
- independent saved-QP oracle: `data/experiments/2026-08-27-phase21-attribution-qp-oracle-v2/`
- configuration: `simulation/mujoco/config/phase21_attribution.json`
- runners: `tools/experiments/audit_weighted_wbc_attribution.py` and `tools/experiments/audit_weighted_wbc_qp_window.py`

The matrix contains baseline; disabled wrench fidelity; each disabled task; and each single task. Every case ran for the full 10 s. Per-tick CSVs include `nudot/tau/lambda/slack`, solver residuals and iterations, active-bound margins/counts, task targets/achieved accelerations/residuals/cost/direction, rolling state, mesh-contact resultant/moment/COP, and reduced generalized-force mismatch. Every case also preserves `H,g,A,l,u`, variable/row scales, pre-solve warm start, candidate, reference wrench, and solver diagnostics for ticks 240–272 inclusive.

## Reference-wrench audit (A)

`reference_wrench` is generated exactly once during `ControllerOracle` initialization. It is the static least-squares equilibrium contact force mapped through the equilibrium contact-point wrench map; it is not recomputed as the robot rolls. The exact 12D value is recorded in the attribution summary and every saved failure-window problem.

Disabling only wrench fidelity leaves all hard constraints and motion/contact tasks unchanged and makes slack unpenalized. It moves the first iteration-limit event only from tick 272 to tick 274 and does not prevent the 10 s failure (`614` failed solver ticks versus baseline `411` after the plant has departed the local domain). At tick 272 its contact/COP/task trends remain of the same kind as baseline. Therefore the static wrench reference can affect the numerical path, but **A is not supported as the primary failure source**.

## Independent QP audit (B)

For baseline ticks 240–272, independent SciPy HiGHS feasibility checks report all 33 saved problems feasible. Maximum reconstructed feasibility violation is `1.32e-14`. Independent SLSQP solves the unchanged convex QP on all 33 ticks with maximum constraint violation `5.75e-14`. The largest audited fixed-ADMM normal-matrix condition number is `317.62`.

At baseline tick 272, the in-run dense NumPy ADMM reaches its frozen 4000-iteration limit, while both independent checks accept the same saved mathematical problem. Thus **B is confirmed as the immediate classification of the tick-272 event: an ADMM numerical/iteration-limit failure on a feasible QP**. This does not make B the initiating physical cause: the plant/contact mismatch and task residuals have already grown before tick 272.

Some ablations have independently infeasible saved problems within ticks 240–272 because their plants have already diverged before or inside that common window. Those cases are not used to generalize the baseline classification.

## Task direction and assembly audit (C)

The previous task switch was not an attribution authority: the main loop never supplied its enabled set, and an empty set was incorrectly replaced with the full task set. The audit fixes both defects and records task-level values every tick.

All five single-task cases produce the intended direction when the target norm is nontrivial:

| Single task | Median direction cosine | Positive fraction |
| --- | ---: | ---: |
| contact acceleration | 0.998 | 0.998 |
| base X | 1.000 | 0.710 |
| height | 1.000 | 0.774 |
| orientation | 1.000 | 0.995 |
| leg posture | 0.991 | 0.988 |

The fractions include late fallen/saturated states; the medians and raw target/achieved vectors reject a systematic sign reversal. This audit does not freeze the final weights or prove global task compatibility, but **C is not supported as a common task sign/order/assembly defect**.

## Mesh-contact generalized-force audit (D)

The validation-only plant audit resolves every wheel-floor mesh contact with `mj_contactForce`, rotates force and contact torque to world axes, accumulates force and moment about each wheel center, computes the normal-load-weighted COP, applies each wrench to the wheel body with `mj_applyFT`, and projects the resulting full generalized force through the same reduced tangent. No plant contact truth is fed to the controller.

The baseline reduced generalized-force mismatch is `0.004` at tick 1, then grows as rolling leaves the equilibrium-calibrated fixed-COP domain:

| Tick | mismatch norm | COP error | rolling slip |
| ---: | ---: | ---: | ---: |
| 200 | 1.320 | 0.0013 m | 0.0090 m/s |
| 240 | 1.413 | 0.0016 m | 0.0335 m/s |
| 250 | 8.211 | 0.0064 m | 0.0812 m/s |
| 270 | 20.258 | 0.0221 m | 0.2442 m/s |
| 272 | 25.699 | 0.0270 m | 0.0439 m/s |

Over ticks 200–272, mismatch correlation is `0.708` with rolling slip, `0.852` with COP error, `0.858` with wrench slack, `0.882` with base-X task residual, and `0.893` with contact-task residual. Its correlation with iteration count is only `0.376`, consistent with the numerical event being downstream rather than the sole initiating mechanism.

The mismatch compares the current plant contact wrench with the QP's current ideal contact force, so it includes closed-loop contact tracking as well as geometric reduction error and is not a direct online model-identification residual. The near-zero tick-1 result, growing COP displacement, two failed point-selector semantics, ablation results, and correlated pre-failure trend together nevertheless support **D as the dominant upstream attribution**.

## Decision

- Keep DG21-01 and DG21-02 at `REWORK`.
- Keep P21-T03 `doing` and P21-T06 `blocked`.
- Do not tune weights, relax gates, raise the iteration limit, replace the frozen solver, integrate Core, or create model v10 from this audit alone.
- The next authorized work remains a separately frozen state-dependent rolling-contact representation derivable from canonical state without MuJoCo-private contact truth, followed by the same local, saved-QP, task-attribution, and 10 s nonlinear gates.

No Phase 21 gate is closed by this audit and no RECORD is permitted.

The authorized representation audit and its required local follow-up have now been executed; see [rolling_contact_representation.md](rolling_contact_representation.md) and [lowest_eight_patch_local_oracle.md](lowest_eight_patch_local_oracle.md). The lowest-eight patch preserves force-resultant representability but fails selector position/Jacobian continuity, so it is rejected before Pfaffian, 78D QP, or solver work. Core integration remains prohibited and the 36-variable solver gates remain historical only.
