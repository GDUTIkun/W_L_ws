# Phase 21 Contact-Centered Wrench Condensation

Date: 2026-08-27  
Verdict: **fixed 6D wrench cone PASS; static/local reduced dynamics FAIL; Phase remains REWORK**

## Contract

The physical support remains the frozen continuous six-point patch.  For each wheel, the
resultant is now expressed in the continuous contact frame and about the analytic contact
center:

```text
w_C = [F_r,F_l,F_n,M_r,M_l,M_n]
```

For point offset `r_i^C` and force `f_i^C=[f_r,f_l,f_n]`,

```text
w_C = sum_i [f_i^C; r_i^C cross f_i^C] = G_C f,
f_n >= 0, |f_r| <= f_n, |f_l| <= f_n.
```

No total normal-force limit, ad-hoc moment box, COP box, torsional coefficient or hard
contact kinematics was added.  The existing three-row soft Pfaffian is unchanged.

## Authoritative evidence

- config: `simulation/mujoco/config/phase21_contact_centered_wrench_oracle.json`
- validator: `tools/experiments/validate_weighted_wbc_contact_centered_wrench.py`
- authoritative run: `data/experiments/2026-08-27-phase21-contact-centered-wrench-oracle-v5/`

The validator refuses a non-empty output directory and hashes its source, merged configs,
model inputs, captures, switch evidence and outputs.  Runs v1-v4 are retained: v1 exposed
a non-finite JSON failure-path bug, v3 exposed an invalid facet-boundary construction, and
v2/v4 were superseded while the independent static representation cross-check was completed.
None changed the physical geometry, friction or acceptance contract.

## Fixed map and cone

Across 1,282 wheel-side geometry evaluations covering equilibrium, Phase-15 workspace,
eight deterministic random samples, rolling ticks 1-271 and old selector-switch
neighborhoods:

| Quantity | Result |
| --- | ---: |
| maximum `r_i^C` / `G_C` element variation | `1.284e-16` |
| maximum `G_C` Frobenius variation | `3.955e-16` |
| rank / nullspace dimension | `6 / 12` |
| minimum singular value | `0.0199332` |
| nonzero condition number | `122.885` |

The pointwise square friction pyramids generate 24 unique wrench rays.  A `F_n=1`
five-dimensional section was enumerated offline with SciPy 1.15.3/Qhull `Qx`: 182
simplicial equations (146 duplicate triangulation planes) consolidate to 36 unique physical
slice facets, with no duplicate or opposite duplicate among the retained normalized rows.
Adding `F_n>=0` gives the fixed 37-row homogeneous contract `H_C w_C <= 0`.  SciPy is
BSD-3-Clause and Qhull uses its permissive Qhull license; both are offline evidence tools,
not authorized production dependencies.

The independent 18D point-force LP, 24-ray V-cone LP and direct 37-row H-cone test agree
on all 1,240 membership cases.  The corpus includes zero, named force/moment/friction
boundary cases, every facet with an outward perturbation, random feasible/infeasible cases,
and all 1,077 truth resultants: original `537/537` plus fresh `540/540`.

## Transform, generalized force and virtual work

World/body-center truth is shifted to the analytic contact center before rotation into the
contact frame.  The independent inverse shift has maximum error `1.17e-13`.  Mapping the
condensed wrench back through the pose-dependent contact frame and material-point spatial
Jacobian agrees with the six-point generalized-force sum to `2.66e-15`; virtual work agrees
to `1.42e-14`.  Nullspace perturbations preserve both resultant wrench and generalized
force.  This proves, for the frozen current-nominal support contract, that pose dependence
lives in the wrench transform/dynamics map rather than in the feasible cone.

No current frozen hard constraint or soft task consumes individual point loads, pressure,
COP or point saturation.  The 12 internal-force coordinates per wheel can therefore be
eliminated from the contact feasible-set contract.  This is not a production-QP approval.

## Static/local reduced-dynamics failure

Only after the cone gates passed, the oracle set `nudot=0`, reconstructed the Phase-15
ideal-closure passive state from canonical base/active coordinates, set full velocity to
zero, and tested

```text
h_r(q,0) = S_r tau + Q_L(q) w_L^C + Q_R(q) w_R^C
```

under the fixed wrench cones and canonical torque bounds `[10,10,2,10,10,2] Nm`.  It used
no weighted task and no total normal-force upper bound.

The primary 18-variable condensed-H LP, an independent 54-variable V-ray LP, and an
independent 42-variable point-force LP agree for all 173 states:

| Representation | Feasible |
| --- | ---: |
| condensed `tau + w_L + w_R` | `122/173` |
| `tau +` two 24-ray coefficients | `122/173` |
| `tau +` two 18D point-force blocks | `122/173` |

All 51 failures are representation-consistent: one deterministic random state and 50
selected reconstructed rolling states spanning ticks 212-271.  Thus this is not a
facet-conversion, internal-force-elimination or HiGHS-status artifact.  Among feasible
states, maximum hard residual is `2.64e-8`, minimum torque margin is `1.462 Nm`, maximum
H-row violation is `7.71e-13`, and maximum recovered point-force witness error is
`2.39e-12`.

## Decision

- The fixed contact-centered 6D wrench cone is accepted for the current nominal six-point
  support model; DG21-02 can close at the contact feasible-set layer.
- Static/local reduced-dynamics feasibility fails, so DG21-01 remains `REWORK` and P21-T03
  remains `doing`.
- A 42D hard-WBC problem is **not** mathematically authorized.  The three-way static result
  also shows that restoring the eliminated point/internal-force variables cannot repair the
  failure.
- P21-T04/T05/T06, solver benchmarking, hard-QP assembly and Core integration remain
  blocked.  The next permitted work is a bounded attribution of the 51 static failures;
  task/solver tuning and contact-geometry changes remain prohibited.
