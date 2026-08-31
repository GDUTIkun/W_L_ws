# Phase46 MuJoCo-Dependent Simulation-Only R2 Repair

> **WARNING — MUJOCO-DEPENDENT SIMULATION-ONLY R2**
>
> This implementation uses current-state MuJoCo internal constraint-response quantities and is intended
> only to close the simulation control loop. It must not be deployed to the real robot unchanged. Before
> hardware deployment, replace this layer with a hardware-valid physical, identified, or otherwise causal
> real-contact realization model.

## Decision

Classification is `H-MUJOCO-R2-HARD-INTEGRATION-OVERCONSTRAINED`. Same-snapshot provenance,
no-future-response, contact/equality partition, native oracle reconstruction, M/P force-dual covariance,
legal reduced rank, contact-force image and pre-solve active-set gates pass. The one simulation-only hard
profile was therefore assembled, but its compatible-H0 QP is `PrimalInfeasible`; `COMP=FAIL` and the
strict stop prevents EQ, AUTH, REAL, SHORT and 10 s.

This is not a hardware-ready or plant-independent R2 result. Default and historical production profiles
remain unchanged; only `kPhase46MujocoContactResponse` consumes the new current-tick payload.

## Same-tick source and law

`Adapter::extractState` performs `mj_forward` before the controller boundary. The R2 simulation adapter
then uses only that pre-command state and same-state scratch forwards:

```text
qpos, qvel, qM, qfrc_smooth,
efc_J, efc_D, efc_aref, efc_type, efc_id,
contact topology and frames.
```

It does not consume post-command/post-step `efc_force`, future `qacc`, future contact force or next-state
measurements. Zero-control and six actuator-column scratch forwards preserve the state and constraint
snapshot. The fixed-current-active-set law is:

```text
f = D (aref - J qacc)
M qacc = qfrc_smooth + B tau + J^T f.
```

The qacc reconstruction error is `1.47787337923e-12`. Contact and equality rows are separately decoded;
their generalized reactions sum to the total coupled reaction with error `2.35922392733e-16`.

## Canonicalization and reduced legality

The native reaction is transformed once from MuJoCo base-body origin `M` to production base-control
reference `P` using the verified force-dual relation, then projected with production `N^T`. Virtual-power
error is `3.55271367880e-15`.

The full 16D hard form is illegal because it reintroduces closure-dual freedom. The legal reduced decision
rows have rank 7, condition number `121.623161993`, and are compressed to seven independent rows before
QP assembly. Current hard-equality rank is 12; rank with R2 is 19, so incremental rank is exactly 7.
The corrected-R1 contact-force image residual is `3.55271367880e-15`.

The pre-solve active-set signature is `d5f6b1c2e9b990a7`; minimum predicted pyramidal row force is
`1.15928411998`, so the candidate does not predict a regime invalidation before solve. No inner
active-set iteration is used.

## Hard integration result and stop

The hard QP returns:

```text
solver:               PrimalInfeasible
primal residual:      0.149973974436
dual residual:        0.000258367913212
stationarity residual: 51978.8558844
```

Thus the legal independent reduced relation remains incompatible with the existing hard/inequality set at
H0. Soft fallback, weights, gains, contact parameters, solver parameters and active-set iteration are all
forbidden, so no workaround was attempted.

```text
COMP:  FAIL
EQ:    NOT ENTERED
AUTH:  NOT ENTERED
REAL:  NOT ENTERED
SHORT: NOT ENTERED
10 s:  NOT ENTERED
```

Evidence: [formal-v1](evidence/automated/r2-mujoco-dependent-simulation-repair-formal-v1/r2-mujoco-dependent-simulation-repair.json)
and [fresh replay-v1](evidence/automated/r2-mujoco-dependent-simulation-repair-replay-v1/summary.json).
