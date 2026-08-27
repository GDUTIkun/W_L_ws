# Phase 21 Continuous Contact Representation Oracle

Date: 2026-08-27  
Verdict: **local representation PASS; production Phase remains REWORK**

## Decision

The rejected lowest-eight selector is not repaired. The new current-nominal candidate is an analytic continuous wheel-ground frame plus a fixed six-point virtual surface patch, derived only from canonical state, Phase-15 passive reconstruction, compiled wheel geometry, the fixed ground normal, nominal radius, and the already frozen `1 mm` support band. MuJoCo contact lists, forces, COP, penetration, slip, and contact count are validation-only outputs and are not representation inputs.

For wheel-axis unit vector `a` and ground normal `n`:

```text
t_r = normalize(a × n)
t_l = n × t_r
r_up = normalize(n - (a·n)a)
p_c = p_mesh_center - radius r_up
```

`[t_r,t_l,n]` is right-handed and preserves controller/world FLU signs. The compiled mesh axial span is `39.9999991 mm` on both sides; its axial midpoint is `+84.8603 um` left and `-84.8603 um` right. With `r=50 mm` and support depth `d=1 mm`, the rolling half-length is the circle-derived `sqrt(2rd-d²)=9.949874 mm` and the lateral half-width is `20 mm`.

The fixed point order is:

1. bottom `(rolling=0, lateral=-20 mm, normal=0)`;
2. bottom `(0,+20 mm,0)`;
3. band edge `(-9.949874,-20,+1) mm`;
4. band edge `(-9.949874,+20,+1) mm`;
5. band edge `(+9.949874,-20,+1) mm`;
6. band edge `(+9.949874,+20,+1) mm`.

The first four-corner candidate used only points 3–6. It is rejected, not tuned: it passes the synchronized fresh corpus but only `534/537` original frozen samples, with maximum residual `6.84945e-4`. Adding the two geometry-required bottom lateral endpoints represents tangential-force moments at the bottom of the circular support band and is the minimum tested geometry-complete surface patch.

## Independent evidence

Authoritative runs:

- continuous representation: `data/experiments/2026-08-27-phase21-continuous-contact-oracle-v6/`
- continuous-center Pfaffian: `data/experiments/2026-08-27-phase21-continuous-pfaffian-oracle-v3/`
- configs: `simulation/mujoco/config/phase21_continuous_contact_oracle.json` and `simulation/mujoco/config/phase21_continuous_pfaffian_oracle.json`
- validators: `tools/experiments/validate_weighted_wbc_continuous_contact.py` and `tools/experiments/validate_weighted_wbc_continuous_pfaffian.py`

Both validators refuse non-empty output directories and hash their configs, source, captures, prior switch evidence, and outputs. Iterative v1–v5 contact and v1–v2 Pfaffian directories are retained; v6/v3 are authoritative after coverage and semantic corrections.

## Geometry and differential gates

Coverage includes equilibrium, the three Phase-15 workspace samples, eight deterministic random in-envelope workspace states (`seed=2106`), rolling ticks 1–271, all 119 old ordered-selector switch events and neighbors, and eleven-point interpolated sweeps through every old switch bracket.

| Gate | Result | Authority |
| --- | ---: | ---: |
| frame orthonormal/determinant error | `1.22e-14` | `<=1e-12` |
| minimum consecutive frame dot | `0.999999521` | positive/no flip |
| minimum fine-sweep frame dot | `0.999999997` | positive/no flip |
| maximum fine-sweep point increment | `8.73e-5 m` | `<=1e-3 m` |
| geometric Jacobian velocity FD | `1.23e-7 m/s` | `<=2e-6 m/s` |
| geometric `Jdot_nu` cross error | `5.01e-7 m/s²` | `<=2e-5 m/s²` |

Velocity FD converges from `1.23e-7` at `1e-4 s`, to `1.22e-9` at `1e-5 s`, to `4.27e-10` at `1e-6 s`. No point membership, slot identity, or frame-sign branch exists in this representation.

Two Jacobians are deliberately distinct:

- `J_geom = dp_virtual/dnu` validates continuous virtual geometry and its bias;
- `J_force` is the instantaneous wheel-body material-point Jacobian evaluated at each virtual force location and is used for physical generalized force/virtual work.

The virtual support points are not claimed to be material trajectories. This distinction prevents a state-dependent geometric selector derivative from being misused as the generalized-force map.

## Force map and truth representability

Each point force is ordered `[rolling,lateral,normal]` in the continuous contact frame, transformed to world FLU by `R_c=[t_r,t_l,n]`, with `|f_r|,|f_l| <= mu f_n`, `f_n>=0`, and frozen `mu=1`.

Across 42 force-map case types and 13,944 evaluations—every point/axis basis, random single-wheel distribution, symmetric/asymmetric load, pure tangent/normal, and moment-producing distribution—the maximum reduced `mj_applyFT` projection error is `1.11e-15` and maximum virtual-work error is `3.00e-15`.

| Corpus | Four band corners | Selected six-point patch | Six-point max residual |
| --- | ---: | ---: | ---: |
| original frozen | `534/537` | `537/537` | `0` |
| synchronized fresh | `540/540` | `540/540` | `0` |

The analytic contact center can be as far as `20.06 mm` from truth COP; that does not fail the patch because load distribution, rather than the geometric center, realizes the resultant. A solved-force COP is non-unique because the point-force map has an internal nullspace, so the evidence does not invent a unique COP error.

## Rank and 6D wrench condensation

For each wheel, `G(q)` maps 18 point-force components to a six-dimensional wrench about the actual wheel/body center. Across all 271 rolling states and twelve workspace states per side (`566` rows):

- rank is always `6`;
- nullspace dimension is always `12`;
- minimum nonzero singular value is `0.0199088`;
- nonzero condition number is `123.186–123.190`.

Thus internal point-force variables are not mathematically required to represent the resultant wrench. At each state the pointwise pyramids project to an exact convex polyhedral 6D wrench cone. However `G(q)` varies with pose (`max sampled change 0.01277`), so a single fixed local H-representation has not been proven. Exact projected constraints, a conservative inner approximation, or a small internal-force basis remains the next mathematical contract decision; this run does not assemble a 66D point-force QP or a 42D condensed QP.

## Pfaffian decision

Force support remains separate from contact kinematics. The candidate contact-acceleration map has exactly three rows per wheel, not one rigid constraint per support point:

```text
A_side(q) = [t_r t_l n]^T J_material(p_c(q), wheel_body) N(q)
```

It represents the `[rolling,lateral,normal]` velocity of the instantaneous wheel material point located at the analytic continuous contact center. It is validated only for the existing compliant-contact **soft acceleration task**. It is not authority for a hard rigid 3D constraint.

Using synchronized captured velocities up to reduced `7.391` and wheel `5.988`, independent frozen-material-point velocity FD has maximum error `1.52e-7 m/s`; the nested independent `Adot_nu` cross-oracle has maximum error `2.84e-6 m/s²`. Canonical positive wheel speed produces positive rolling velocity on both sides (`0.049966/0.050103 m/s` for unit wheel speed). Actual bias reaches `9.864 m/s²` in late old-switch-region states, but it is smooth and independently reproduced; physical high-velocity bias is not clipped or confused with the rejected selector's undefined jump.

## Status recommendation

The continuous frame, six-point force support, geometric differential map, physical generalized-force map, truth resultant envelope, and three-row soft Pfaffian local semantics pass their current oracles. Nevertheless:

- DG21-01/02 remain `REWORK` until the exact 6D wrench constraint/internal-force contract and new static reduced-dynamics gate pass;
- P21-T03 remains `doing`;
- P21-T04/T05/T06 and Core integration remain blocked;
- no hard-QP or solver benchmark is authorized yet;
- the next allowed work is only the 6D projected-wrench contract plus static/local dynamics oracle.

No task/solver tuning, online contact truth, public schema, production Core, or `RECORD.md` was created.

## Contact-centered condensation follow-up

The next independent oracle moved the resultant origin to the analytic contact center and
expressed it in the continuous contact frame.  It proved that `G_C` is fixed to numerical
precision, accepted an exact fixed 24-ray/37-row 6D wrench cone, and preserved all original
`537/537` and fresh `540/540` truth resultants.  Generalized-force and virtual-work checks
also passed, so the 12 internal-force coordinates per wheel are not required by the frozen
contact feasible-set contract.

The same run rejected the new static/local reduced dynamics: condensed-H, V-ray and direct
point-force formulations all find only `122/173` states feasible.  Consequently the cone
result does not authorize a 42D hard-QP candidate.  See
[contact_centered_wrench_condensation.md](contact_centered_wrench_condensation.md).
