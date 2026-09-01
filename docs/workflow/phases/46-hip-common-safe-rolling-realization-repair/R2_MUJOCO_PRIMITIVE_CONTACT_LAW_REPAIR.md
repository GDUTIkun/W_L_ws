# Phase46 MuJoCo Primitive Contact-Law W5 Closure

> **WARNING — MUJOCO-DEPENDENT SIMULATION-ONLY R2.** This profile consumes current-state MuJoCo
> contact internals. It is not hardware-ready and must not be deployed unchanged.

## Decision

Classification is `I-EQ-FAIL`. The primitive contact-law integration, W1–W6, complete 42D witness and
`COMP` pass. The first mandatory failure is the frozen EQ slack threshold, so AUTH, REAL, SHORT and
10 s are not entered. Phase46 remains `review/REWORK`; no RECORD is created.

## W5 root cause and closure

The runtime used `X_PM` for the production-reference-to-MuJoCo acceleration lift and used its transpose
as the generalized-force dual. The authoritative edge is `X_MP`. Fixing only this transform direction
closes all three independently assembled reduced row operators:

```text
historical operator residual: 7.656794677961396
historical offset residual:   7.2009163679271335
fixed operator residual:      1.0658141036401503e-14
fixed offset residual:        2.6645352591003757e-15
max static A/B/C residual:    8.881784197001252e-16
```

The general affine acceleration transform remains in the implementation. At the frozen H0 state,
`Xdot*nu=0` because velocity is zero; this does not authorize deleting the general bias term.

## H0 witness and ordered stop

W1–W6 pass; the compressed decision row rank and incremental hard-equality rank are both 10. The 42D
witness is `SOLVED` with hard residual `3.623378202098832e-9`, minimum inequality margin
`0.2197161714633595`, minimum torque margin `1.9990801609079853`, R1 residual
`1.2258392916278746e-14`, and primitive-law raw residual `2.74192713867194e-7`. The latter is below the
existing controller/test contract of `1e-6`; the global hard metric is normalized separately.

Candidate row-force margin is positive at `3.5496361436896655`. The old negative row force obtained by
projecting an arbitrary diagnostic MuJoCo `qacc` is retained only as a diagnostic and is not an
active-set pre-assembly veto.

`COMP` therefore passes. `EQ` fails only because maximum normalized slack is
`0.05850370867784012 > 0.05`. The runtime's independent rollout gate stops after tick 0 for the same
reason. Consequently a one-row CSV from a 223/1000-tick request is an ordered stop, not a completed
rollout; SHORT and 10 s remain `NOT ENTERED`.

Evidence: [formal-v1](evidence/automated/r2-mujoco-primitive-contact-law-repair-formal-v1/r2-mujoco-primitive-contact-law-repair.json)
and [fresh replay-v1](evidence/automated/r2-mujoco-primitive-contact-law-repair-replay-v1/summary.json).

## Wrench-slack closure addendum — mandatory request-feasibility stop

This addendum supersedes only the earlier `I-EQ-FAIL` root-cause label; the W1–W6, witness and COMP
results above remain unchanged. The 12D quantity is ordered left then right as
`[Fx,Fy,Fz,Tx,Ty,Tz]`, expressed in controller-body FLU, with moments about the corresponding
wheel-body origin and sign wheel-follower-on-leg/base. Its scales are `[50,50,50,2.5,2.5,2.5]` per
side. The implemented identity
`realized - reference - slack = wrench_residual` reconstructs exactly (`0.0` max error).

The point-realizable baseline has maximum normalized slack `0.001522220395389018`; primitive R2 has
`0.05850370867784012`. The R2 dominant channel is right `Tx`: its normalized slack changes from
`1.5222939650929801e-6` to `-0.05850370867784012`, a delta of
`-0.05850523097180521` (physical delta `-0.14626307743304376 Nm`).

The mandatory fixed-H0 feasibility LP used the same 42 normalized variables, all 22 hard equalities
(12 dynamics plus 10 primitive rows), torque/cone/acceleration inequalities, corrected rank-5 wrench
projectors, and the full rank-12 realized-interaction-wrench affine operator. Adding
`W_realized = W_reference` is infeasible. An independent minimax LP finds the smallest possible
normalized infinity-norm wrench deviation to be `0.07832043067340007`; its hard-equality residual is
`2.4952757450346064e-11` and minimum inequality margin is `0.0`.

Therefore classification is `A-WRENCH-REFERENCE-NOT-PRIMITIVE-FEASIBLE`, with an upstream
request-realizability conflict. Per the mandatory branch, soft-objective inventory, KKT attribution,
one-task ablation and repair are not entered. No weight, gain, threshold, primitive law, W5 or R1 was
changed. AUTH, REAL, SHORT and 10 s remain `NOT ENTERED`; the historical one-row 223-tick invocation
remains `NOT COMPLETED — EQ stopped at tick 0`. Formal and fresh replay decisions are byte-identical:
[closure formal](evidence/automated/r2-wrench-slack-closure-formal-v1/r2-mujoco-primitive-contact-law-repair.json)
and [closure replay](evidence/automated/r2-wrench-slack-closure-replay-v1/r2-mujoco-primitive-contact-law-repair.json).
