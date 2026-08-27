# Phase 21 Base/Contact Compatibility Closing

Date: 2026-08-27  
Verdict: **PASS — seven apparent nominal blockers are static-gate semantics cases (A)**

## Scope and authority

This closing audit keeps the frozen `12D` reduced coordinates, passive reconstruction,
continuous six-point patch, analytic contact frame/center, `mu=1`, fixed 37-row H-cone,
canonical torque bounds and current nominal plant parameters unchanged. It does not assemble
a QP, tune a solver/task/contact parameter, or modify Core.

Authoritative inputs and outputs:

- prior static authority: `data/experiments/2026-08-27-phase21-contact-centered-wrench-oracle-v5/`
- rolling capture: `data/experiments/2026-08-27-phase21-patch-local-capture-v2/`
- config: `simulation/mujoco/config/phase21_base_contact_closing.json`
- validator: `tools/experiments/validate_weighted_wbc_base_contact_closing.py`
- result: `data/experiments/2026-08-27-phase21-base-contact-closing-v1/`

The manifest hashes all configs, the validator, prior static summary/manifest/static result,
the rolling capture and both generated JSON files. The matched controls are `equilibrium`,
`random_03`, `tick_210` and `tick_211`.

## Contract decision

Phase 15 explicitly validates closed-chain assembly, passive reconstruction, branch continuity,
Jacobians, virtual work, conditioning and a kinematic workspace. Contact solve, friction, slip and
floating-base/contact dynamics were out of its scope. Its componentwise envelope therefore means
**reconstruction-valid**, not **guaranteed static support**.

Phase 20 freezes one upright bilateral static equilibrium and a separately generated admissible
dynamic perturbation/formal envelope. It does not declare every Phase 15 configuration, nor every
configuration visited by a rolling trajectory, to be zero-velocity static-admissible. Consequently,
the Phase 21 raw static corpus had mixed model/reconstruction coverage with a stronger property that
the earlier contracts never promised.

The corrected prior-contract gate is:

- reconstruction/model gates apply throughout the Phase 15 frozen workspace;
- zero-velocity static support is mandatory for the Phase 20 equilibrium and for any future state
  produced by an explicitly frozen static-admissible generator;
- rolling trajectory configurations are checked by the dynamic reduced equation and plant safety,
  not presumed to admit `nu=0, nudot=0`;
- workspace membership alone never upgrades a configuration to static-admissible.

This is a clarification of the prior Phase 15/20 scopes, not a post-hoc workspace shrink. All 173
states remain in the coverage corpus and fail-closed behavior is unchanged.

## Required wrench and cone distance

For each state the cone is first removed while torque bounds remain frozen. A deterministic
normalized-L1 epigraph LP selects one exact required-wrench witness. Force is normalized by half
the robot weight and moment by that force scale times the `0.05 m` wheel radius. Each wrench is then
projected independently into the frozen cone in normalized L2 and L-infinity metrics.

All seven blockers violate the same physical facet, H-row 28:

```text
-0.00099995 Fr - 0.00994938 Fn + 0.99995000 Ml <= 0
```

Thus the obstruction is the positive local pitch-moment support bound, weakly coupled to rolling
force and normal load. It is not lateral friction, rolling friction, unilateral normal, roll moment
or torsional moment. `random_04` through `tick_213` violate it on the left only; ticks 217–220 violate
it on both wheels.

| State | left facet violation | right facet violation | nearest L2 left | nearest L2 right | frozen full residual |
| --- | ---: | ---: | ---: | ---: | ---: |
| `random_04` | 0.144070 | 0 | 0.089529 | 1.51e-8 | 0.014205 |
| `tick_212` | 0.029001 | 0 | 0.018022 | 2.21e-8 | 0.003344 |
| `tick_213` | 0.045118 | 0 | 0.028038 | 1.52e-8 | 0.025375 |
| `tick_217` | 0.116175 | 0.062439 | 0.072194 | 0.038801 | 0.129792 |
| `tick_218` | 0.137176 | 0.083049 | 0.085244 | 0.051609 | 0.161202 |
| `tick_219` | 0.158152 | 0.105034 | 0.098279 | 0.065271 | 0.193849 |
| `tick_220` | 0.180596 | 0.128509 | 0.112227 | 0.079858 | 0.228798 |

The L2 values are dimensionless under the frozen normalization. `cases.json` preserves required
and nearest L2/L-infinity wrenches, component corrections, active facets, left/right normal-load
redistribution and the resulting signed 12-row equilibrium residual.

## Floating-base versus active rows

Every blocker is feasible when only the six active generalized-force rows are enforced and
infeasible when only the six floating-base rows are enforced. The full 12-row problem therefore
first fails at floating-base force/moment balance; active/passive reduction does not introduce a
later independent failure. Matched controls pass their full frozen reduced problems.

## Same-configuration plant static oracle

The validation-only plant oracle uses exactly the reconstructed `q`, zero velocity and the actual
MuJoCo mesh-contact topology. It does not use the trajectory's `mj_contactForce` result as an
answer. Instead it solves full generalized-force balance over bounded canonical actuator torque,
free closed-chain constraint multipliers and per-mesh-contact `[Fn,Ft1,Ft2]` forces under the
compiled `mu=1` pyramids. Unit force columns are independently built with `mj_applyFT`.

| State | plant static | mesh contacts | contacted sides | minimum full-force L-inf residual |
| --- | --- | ---: | --- | ---: |
| `random_04` | infeasible | 2 | right only | 8.03571 |
| `tick_212` | infeasible | 4 | bilateral | 0.341181 |
| `tick_213` | infeasible | 4 | bilateral | 0.363461 |
| `tick_217` | infeasible | 4 | bilateral | 0.465098 |
| `tick_218` | infeasible | 5 | bilateral | 0.496334 |
| `tick_219` | infeasible | 4 | bilateral | 0.524162 |
| `tick_220` | infeasible | 4 | bilateral | 0.556210 |

The equilibrium matched control is plant-static feasible with full residual `1.47e-13`; its plant
contact wrench and `N^T` projection are preserved for plant-versus-reduced comparison. The two
rolling controls and `random_03` are reduced-static feasible but plant-static infeasible. This is
expected: a reduced patch may admit a static force distribution at a configuration that the actual
mesh topology does not admit, and a rolling configuration was never promised to be static.

## Full-to-reduced consistency and dynamic probe

For every case `h_r=N^T h_full`, `S_r=N^T S_full` and the contact wrench map is formed from the same
`mj_applyFT`/virtual-work convention already closed by DG21-02. Closure residual is at most
`1.85e-14 m`; passive condition number is `7.22–7.30`, so neither closure nor branch conditioning
explains the failures.

For rolling ticks 210, 211, 212, 213 and 217–220, the validation-only dynamic probe restores captured
velocity, estimates full acceleration by centered 10 ms velocity difference, projects
`M qdd + h` through the reconstructed tangent, and compares it against captured canonical torque
plus captured plant resultant transformed at the frozen analytic centers/frames. All `8/8` pass the
pre-frozen absolute/relative gates `1.5` and `0.03`; worst values are `1.39348` and `0.0221514` at
tick 218. The static failures therefore do not reproduce as dynamic-equation failures.

## `random_04`

`random_04` is inside the Phase 15 componentwise envelope (`max ratio=0.953594`), has closure
`1.85e-14 m` and passive condition number `7.30467`. Nevertheless its analytic left center is
`11.83 mm` above the ground while the right center is `10.03 mm` below it; the compiled plant finds
two right-wheel contacts and no left-wheel contact. It is not a physically bilateral-support
configuration. This is exactly the distinction between a valid closed-chain reconstruction sample
and a static-admissible support sample.

## Classification and gate decision

| State | Primary classification | Evidence |
| --- | --- | --- |
| `random_04` | A — static-gate semantics | reconstruction-valid, unilateral plant contact, plant-static infeasible; no prior static-support claim |
| `tick_212` | A — static-gate semantics | rolling configuration, plant-static infeasible, dynamic probe PASS |
| `tick_213` | A — static-gate semantics | rolling configuration, plant-static infeasible, dynamic probe PASS |
| `tick_217` | A — static-gate semantics | rolling configuration, plant-static infeasible, dynamic probe PASS |
| `tick_218` | A — static-gate semantics | rolling configuration, plant-static infeasible, dynamic probe PASS |
| `tick_219` | A — static-gate semantics | rolling configuration, plant-static infeasible, dynamic probe PASS |
| `tick_220` | A — static-gate semantics | rolling configuration, plant-static infeasible, dynamic probe PASS |

No state is classified B, C or D. All required-witness, cone-projection, reduced-residual,
plant-oracle, reconstruction, dynamic and classification gates pass. DG21-01 may close by Route 1;
P21-T03 is complete.

The 42D contact-wrench hard-QP is now **authorized only as the next mathematical candidate**.
No 42D QP, solver, task, Core, nonlinear or hardware result exists yet; P21-T04 must freeze and
validate that problem before downstream gates can move.
