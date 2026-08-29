# Phase 30 v2 structured-formulation evidence

Date: 2026-08-29

## Frozen candidate audit

The supplied Phase 31 design was executed as a non-overwriting Phase 30 v2 route. Redundant A3 and
instrumentation-only B1 were removed because they are mathematically identical to A1 and baseline;
nonlinear B3 was excluded. Before primary results, the route froze A1/A2, B2 with `rho=±0.25`, and
B4 differential-only wheel-rate cost. No candidate or parameter was added after the smoke result.

- Method: [`phase30_structured_formulation_v2.json`](../../../../../simulation/mujoco/config/phase30_structured_formulation_v2.json), SHA-256 `975ec2eccd8033275bc7bfaf1e9e60b3881649c8cb7c3b80b81850d09b213244`.
- Evaluator: [`run_phase30_structured_formulation.py`](../../../../../tools/experiments/run_phase30_structured_formulation.py), SHA-256 `c41f3f5ee0d6468c36ce84f95c3c629e35756da40e1cc6d8377e1601905e4996`.
- Frozen interpreter probe: Python 3.10.20, NumPy 2.2.6, SciPy 1.15.3, MuJoCo 3.7.0,
  CasADi 3.7.2 and importable acados_template. `py_compile` and JSON parse passed.

Full-matrix all-one running/terminal cost reproduced Phase 29 T0/T1 production and converged `u0`
and converged objective exactly. This closes the matrix-setter and independent objective oracle gate.

## Positivity, conditioning and numerical health

Every candidate is PSD. A1/A2 terminal matrices have expected nullity 2/1; B4 running matrix has
expected common-wheel-rate nullity 1. B2± minimum running eigenvalue is `0.93246`; positive-subspace
condition is `21448.6` versus baseline `20000`. Across candidate samples, maximum stationarity is
`8.77e-10`, maximum feasibility residual `5.34e-9`, and maximum cost recomputation error `5.33e-15`.
Thus the causal failures are not R31-E conditioning, feasibility or KKT failures.

## T0 matrix

| Candidate | RTI pitch score | `D_ry` range | `D_omega_y` range | x/vx guards | Result |
| --- | ---: | ---: | ---: | --- | --- |
| A0 baseline | `+0.19687` | `-116.48..-115.28` | `-17.92..-17.75` | negative | control FAIL |
| A1 no terminal x/vx | `-0.29512` | `-47.97..-47.48` | `-12.55..-12.44` | negative | FAIL |
| A2 velocity-only terminal | `-0.28014` | `-50.56..-50.06` | `-12.78..-12.67` | negative | FAIL |

A1/A2 confirm the Phase 29 RTI counterfactual but do not establish restorative local feedback: the
pitch derivatives and all longitudinal guards fail at ticks 54/56/58. No T0 candidate exists;
classification is `R31-A_terminal_structural_candidates_fail`.

## T1 matrix

| Candidate | RTI longitudinal score | tick 44 `a_x` | `D_ry` approximate range | `D_omega_y` approximate range | Result |
| --- | ---: | ---: | ---: | ---: | --- |
| B0 baseline | `+0.001136` | `-0.01268` | `-115.13..-114.20` | `-17.65..-17.56` | control FAIL |
| B2 rho `+0.25` | `+0.001047` | `-0.01170` | `-115.58..-114.63` | `-17.68..-17.58` | FAIL |
| B2 rho `-0.25` | `+0.001226` | `-0.01367` | `-114.68..-113.77` | `-17.63..-17.53` | FAIL |
| B4 differential-only wheel rate | `+0.001028` | `-0.01149` | `-116.29..-115.30` | `-17.71..-17.61` | FAIL |

All candidates retain a non-restorative production action. Ticks 44/46 retain negative longitudinal
acceleration, and pitch angle/rate derivatives remain anti-corrective. No T1 candidate exists;
classification is `R31-B_cross_state_structural_candidates_fail`.

## Replay and gate consequence

- Formal: [`phase30-structured-v1`](automated/phase30-structured-v1/summary.json).
- Fresh replay: [`phase30-structured-v2`](automated/phase30-structured-v2/summary.json).
- Both semantic files are byte-identical. Matrix SHA-256 is
  `12f1739c8d86cb22d08c73e1e029b7cfed101b8e80ed304b1a44a74238f96f4f`; summary SHA-256 is
  `345e3f87afade1f4b3091ca6577bc268f6a2552e6d17ad8eec02c9727ad88f16`.
- Existing-output replay was rejected before solver construction.

Because neither branch produced a candidate, isolated/combined closed-loop, static artifact,
T2/T3 and production regressions are blocked by the frozen route. Production solver, C++ audit,
WBC, model, horizon, lifecycle, thresholds and public interfaces remain unchanged. This evidence
rejects this bounded candidate set; it does not prove that every possible structured cost must fail.
