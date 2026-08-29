# Phase 30 repair-causality evidence

Date: 2026-08-29

## Authority and method

- Authority: Phase 29 formal-v1, Phase 28 `phase28-drift-attribution-v5`, Phase 27 OCP v2.
- Evaluator: [`run_phase30_nmpc_formulation_repair.py`](../../../../../tools/experiments/run_phase30_nmpc_formulation_repair.py), SHA-256 `875381039be8dd31ef404179c0e77d8a89b77f9e265974d0671fa3d2fb1c3444`.
- Method: [`phase30_nmpc_formulation_repair_v1.json`](../../../../../simulation/mujoco/config/phase30_nmpc_formulation_repair_v1.json), SHA-256 `21c44896bc120f6a6d86e843330789d4d00605a76fd1d66bea7e0e6b43713fe8`.
- Runtime cost parameterization: `Q_run=diag(q*s_run)`, `Qe=diag(10*q*s_terminal)` through the existing acados stage `W/W_e` setters. Input cost, dynamics, horizon, bounds, references and lifecycle are unchanged.
- Grid: `[0, 0.125, 0.25, 0.5, 0.75, 1]`; no interpolation or post-hoc points.

## Preflight and baseline parity

The repository interpreter probe passed with Python 3.10.20, NumPy 2.2.6, SciPy 1.15.3,
MuJoCo 3.7.0, CasADi 3.7.2 and importable acados_template. `py_compile` and JSON parsing passed.
An initial `/tmp` smoke without the acados library path stopped before output creation; it was an
environment setup failure and is not control evidence. Formal runs used
`ACADOS_SOURCE_DIR=/home/t/opt/acados` and `LD_LIBRARY_PATH=/home/t/opt/acados/lib`.

The all-one scale profile reproduced Phase 29 exactly:

| Case | production `u0` max error | converged `u0` max error | converged objective error | source prefix request error |
| --- | ---: | ---: | ---: | ---: |
| T0 | 0 | 0 | 0 | `7.77e-16` |
| T1 | 0 | 0 | 0 | `7.22e-16` |

This closes the runtime running/terminal decomposition and legacy-profile parity gate. Production
C++ and the static generated artifact were not changed because both causal screens failed before
integration authorization.

## T0 terminal-x screen

At `alpha=0`, the current-point pitch products are corrective, but the matching local derivatives
are anti-corrective at every frozen update. The same profile also produces anti-corrective x/vx net
action, so the x-only terminal intervention does not satisfy either the local robustness or
longitudinal guard.

| Update | `C_theta` | `D_theta` | `C_omega_y` | `D_omega_y` | `C_x` | `C_vx` |
| ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| 54 | `0.01091` | `-71.367` | `0.04705` | `-14.697` | `-4.85e-4` | `-3.97e-3` |
| 56 | `0.02963` | `-71.114` | `0.16017` | `-14.646` | `-6.81e-4` | `-5.30e-3` |
| 58 | `0.06334` | `-70.736` | `0.36920` | `-14.572` | `-9.21e-4` | `-6.91e-3` |

All six alpha grid values fail the frozen gate. Classification:
`R30-A_terminal_scalar_insufficient`; no `alpha_star` exists.

## T1 running-attitude screen

At `beta=0`, the direct weight counterfactual does not reproduce the Phase 29 attitude-state-removal
direction at the authority update. This distinction is decisive: setting an error to its reference
is not equivalent to removing its running cost. At tick 44, x/vx products remain anti-corrective,
and pitch angle/rate derivatives remain strongly anti-corrective.

| Update | `C_x` | `D_x` | `C_vx` | `D_vx` | `C_theta` | `D_theta` | `C_omega_y` | `D_omega_y` |
| ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| 42 | `3.28e-4` | `1.048` | `1.58e-3` | `0.531` | `-0.02957` | `-113.117` | `-0.05591` | `-16.266` |
| 44 | `-6.93e-5` | `1.037` | `-3.16e-4` | `0.525` | `-0.01125` | `-112.389` | `-0.05561` | `-16.207` |
| 46 | `-1.79e-5` | `1.030` | `-7.84e-5` | `0.522` | `-0.01718` | `-111.930` | `-0.08459` | `-16.173` |

All six beta grid values fail; in particular no nonzero eligible beta exists. Classification:
`R30-B_attitude_scalar_insufficient`; no `beta_star` exists.

## Replay and non-overwrite

- Formal v1: [`phase30-direct-weight-v1`](automated/phase30-direct-weight-v1/summary.json).
- Fresh replay v2: [`phase30-direct-weight-v2`](automated/phase30-direct-weight-v2/summary.json).
- `direct_weight_sweep.json` is byte-identical across v1/v2, SHA-256
  `2e39c06e53627b20a50942190e84cfcee0da82ee6cd43a5a69acd4e5d7f06fe8`.
- `summary.json` is byte-identical across v1/v2, SHA-256
  `d87be215737755dc914b1ffea4ca969ce9d73ce08cd498b6f665f7690270c261`.
- Reusing the existing formal-v1 output root was rejected before solver construction and write.

## Gate consequence

DG30-02 and DG30-03 are REWORK. Per the frozen isolated-before-combined rule, T06–T11 and
DG30-04–06 are blocked and were not executed. No production weights, solver wrapper, generated
artifact, WBC task, fault behavior or public interface changed. The evidence is sufficient to reject
these two scalar hypotheses, not to approve a broader cost structure or a particular replacement.
