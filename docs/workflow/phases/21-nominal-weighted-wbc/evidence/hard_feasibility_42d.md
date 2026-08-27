# Phase 21 42D Layered Hard Feasibility

Date: 2026-08-27  
Scope: P21-T05 only; no weighted-task, nonlinear-holdout, Core, ROS, hardware, or NMPC authority

## Decision

DG21-04 is closed for the current nominal profile. The frozen 42D hard problem
is feasible at each cumulative layer, and the Phase-20 equilibrium has a
feasible zero-acceleration solution after all 104 hard rows are present.

The current authoritative MuJoCo model defines all six active joints with
`limited=false` and `range=[0,0]`. P21-T05 therefore excludes state-dependent
joint-position/velocity protection instead of inventing an unapproved joint
range or look-ahead rule. The frozen componentwise 12D acceleration box and the
Phase-15 reconstruction/workspace fail-closed checks remain the applicable
protections. A future joint-range profile requires its own design and evidence.

## Frozen layered audit

`validate_weighted_wbc_hard_layers_42d.py` imports the P21-T04 builder and
independent HiGHS/SLSQP oracle. It constructs each 104-row problem once and only
slices the cumulative frozen row order:

| Layer | Rows | Added contract | Feasible cases |
| --- | ---: | --- | ---: |
| dynamics | 12 | reduced rigid-body dynamics equality | 32/32 |
| torque | 18 | six canonical torque boxes | 32/32 |
| cone | 92 | left/right fixed 37-row contact-centred H-cones | 32/32 |
| acceleration | 104 | componentwise 12D acceleration box | 32/32 |

The corpus is unchanged from P21-T04: four workspace cases and 28 selected
rolling dynamic ticks. Earlier rows are preserved byte-for-byte by prefix
slicing; no layer rebuilds or relaxes a prior constraint. Because the shared
SciPy wrapper represents an empty inequality set as a one-dimensional array,
the equality-only oracle call appends a logged `0*x <= 0` row. This numerical
adapter does not alter the recorded 12-row physical layer or its feasible set.

Across all 128 layered QPs, the worst independent-oracle bound,
stationarity, and complementarity residuals were respectively
`5.47479e-14`, `1.32314e-13`, and `2.62388e-17`. The worst recomputed physical
dynamics residual was `6.49703e-13`. Physical torque, cone, and acceleration
violations were all zero, and the unused future-fidelity slack remained exactly
zero.

The tightest layer-specific margins were:

- torque layer: `1.72868 N·m` at `dynamic_tick_271`;
- cone layer: `1.78191e-14` at `dynamic_tick_271`;
- acceleration layer: `1.10246` in the applicable acceleration unit at
  `dynamic_tick_271`.

At the cone and full layers, `dynamic_tick_271/right_cone_8` was the sole row
within the frozen `2e-7` scaled active-row tolerance. This is an indexed facet
attribution only; P21-T05 does not reinterpret or modify the already frozen
37-row cone.

## Static equilibrium gate

The full 104-row `workspace_equilibrium` problem was augmented with 12
equalities fixing `nudot=0`, producing 116 rows. HiGHS and the independent
minimum-scaled-norm QP both succeeded:

- bound/stationarity/complementarity residual:
  `1.89108e-16 / 9.28959e-16 / 0`;
- physical dynamics residual: `2.37769e-15`;
- left/right normal force: `31.57222 / 31.54924 N`;
- minimum contact-cone margin: `0.310102`;
- minimum torque margin: `1.99854 N·m`;
- acceleration violation and fidelity slack: `0 / 0`.

Only the Phase-20 equilibrium is required to be statically admissible here.
The seven previously attributed rolling states remain dynamic corpus cases;
their same-state static infeasibility is the already closed DG21-01
static-gate semantics and is not reclassified as a hard-QP failure.

## Reproduction and authority

Commands:

```bash
./.venv/bin/python -m py_compile \
  tools/experiments/validate_weighted_wbc_hard_layers_42d.py
./.venv/bin/python \
  tools/experiments/validate_weighted_wbc_hard_layers_42d.py
git diff --check
```

The non-overwritten formal result is
`data/experiments/2026-08-27-phase21-hard-layers-42d-v1/`. Its manifest hashes
the validator, inherited and local configs, model/contact inputs, Phase-20
equilibrium, dynamic capture, and both result JSON files. The recorded runtime
was Python 3.10.20, MuJoCo 3.7.0, NumPy 2.2.6, and SciPy 1.15.3.

This evidence closes P21-T05 and unlocks P21-T06. It does not freeze weighted
tasks, weights, standing-wrench fidelity, nonlinear 10 s behavior, or any
production implementation.
