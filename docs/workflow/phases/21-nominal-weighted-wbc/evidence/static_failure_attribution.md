# Phase 21 Static-Failure Attribution

Date: 2026-08-27  
Verdict: **attribution PASS; nominal static gate FAIL; Phase remains REWORK**

## Question and scope

This audit explains the 51 infeasible states reported by the contact-centered static oracle.
It does not change the frozen six-point geometry, `mu=1` friction pyramids, canonical torque
bounds, reduced dynamics, or acceptance thresholds.  Torque scaling, friction scaling and
constraint removal are counterfactual probes only.

Authoritative inputs and outputs:

- baseline: `data/experiments/2026-08-27-phase21-contact-centered-wrench-oracle-v5/`
- config: `simulation/mujoco/config/phase21_static_attribution.json`
- validator: `tools/experiments/validate_weighted_wbc_static_attribution.py`
- authoritative result: `data/experiments/2026-08-27-phase21-static-attribution-v9/`

Run v8 first closed the attribution gates.  Fresh non-overwrite run v9 reproduces byte-identical
`summary.json` and `cases.json` and is the current authority.

The validator reconstructs exactly the same 173 states and reproduces the original
`122 feasible / 51 infeasible` classification.  Its source, configs, authoritative inputs
and outputs are hashed in the run manifest.

## Frozen-workspace audit

The Phase-15 sample extrema define the existing componentwise base-rotation and canonical
active-joint envelope.  This is a bounding-box coverage check, not a new or enlarged
workspace.  States outside it must already fail closed under the reduced-model contract.

| Population | Total | Feasible | Infeasible |
| --- | ---: | ---: | ---: |
| raw static corpus | 173 | 122 | 51 |
| inside frozen componentwise envelope | 129 | 122 | 7 |
| outside frozen componentwise envelope | 44 | 0 | 44 |

The maximum out-of-envelope component ratio is `15.6073`.  The 44 coverage cases are valid
fail-closed tests, but they are not nominal-feasibility blockers.  The seven nominal
blockers are `random_04` and rolling reconstruction ticks `212`, `213`, and `217`-`220`.

## Bounded counterfactuals

For every state the audit independently solves the condensed-H and direct point-force
minimum uniform torque-scale problems, the exact problem with the cone removed, the exact
problem with torque bounds removed, a point/V cross-checked minimum-friction search, and
the minimum equality `L_inf` residual under the frozen cone and torque bounds.

For all seven in-envelope failures:

- both minimum torque-scale formulations remain infeasible even when the uniform scale is
  unbounded; removing torque bounds therefore does not repair the state;
- removing the contact cone repairs every state;
- the counterfactual minimum `mu` ranges from `1.31711` to `19.47584`, with median
  `15.34661` and p90 `18.60230`;
- the minimum equality residual ranges from `0.00334357` to `0.228798`, with median
  `0.129792` and p90 `0.207829`.

Dominant residual rows are base lateral force twice, base vertical force twice, base roll
moment twice, and left-knee generalized force once.  Six of seven dominant rows are
floating-base equilibrium rows.  Combined with the unbounded-torque result, this attributes
the nominal obstruction to contact-cone/base-static compatibility rather than actuator
limits or a QP solver failure.

The very large counterfactual friction values are diagnostic witnesses, not permissible
tuning.  Frozen `mu=1` remains unchanged.  Among the 44 out-of-envelope cases, 24 have a
finite cone-removal witness and 20 do not within the bounded `mu<=64` search/direct LP;
those classifications do not affect the nominal conclusion.

## Decision

- The attribution gate passes, but the physical static gate does not: seven nominal states
  remain infeasible.
- DG21-01 remains `REWORK`; P21-T03 remains `doing`.
- DG21-02 remains closed only for the exact contact feasible-set condensation.  That result
  does not establish compatibility with the reduced static dynamics.
- Raising friction, relaxing the cone, changing torque bounds, authorizing the 42D hard QP,
  tuning tasks/solver, or integrating Core is not authorized.
- The next allowed work is confined to the reduced base-equilibrium/contact compatibility
  that produces these seven failures.
