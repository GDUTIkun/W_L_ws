# Stage-R Affine Reaction Map Provenance / Reference Attribution

## Decision

Classification is `A-DIAGNOSTIC-STAGER-REFERENCE-MIX-CLOSED`. The historical Stage-R failure was a
diagnostic-only producer/consumer reference mismatch. No production controller or QP code changed,
and the result does not authorize R2 or create an R2 re-authorization candidate.

## Provenance and affine semantics

`Qc0/Qct` are built only inside
`run_phase46_r2_contact_response_reauthorization.py::main` from the fixed-H0 MuJoCo active-set Schur
block. They are consumed by that script's diagnostic Stage-R extra equality and later attribution scripts.
The production `WeightedWbcProblem::assemble` uses independent aggregate wrenches in reduced dynamics,
wrench cones and soft tasks; it contains no equivalent constitutive affine reaction constraint.

The native affine origin is `tau=0`:

```text
Qc(tau) = Qc0 + Qct*tau
Qc0 = fixed-state/fixed-active-set reaction with actuator torque zero
Qct = dQc/dtau at the frozen H0 state and active set
```

This is not an H0-centered `Qc,H0 + Qct*(tau-tau0)` relation.

## Exact wrong edge

The producer creates `Qc0_M/Qct_M`. The historical code correctly transformed these to production
coordinates as `Qc0_P=X_MP^T Qc0_M` and `Qct_P=X_MP^T Qct_M`, but the consumer left
`Aw_full*W_prod` in `M` generalized-force coordinates. The first mixed edge was therefore:

```text
Aw_full(M) * W_prod  ==  Qc0_P + Qct_P*tau
```

The missing transform is exactly one left multiplication by `X_MP^T` after wrench-to-generalized-force
mapping, or equivalently keeping both sides in `M`. It acts only on generalized-force axes.

Fresh covariance checks give zero `Qc0` residual and per-column `Qct` residual at most `5.55e-17`.
The historical mismatch is reproduced as:

```text
offset frame contribution: 1.52219637586
slope frame contribution:  3.31444766252
total dominant residual:    4.83664403838
```

## Corrected diagnostic replay

The diagnostic-only correction `Aw_P=X_MP^T Aw_M` closes the H0 map at `1.9762e-14`. With state,
QP, aggregate representation, tasks, gains, weights and R1 unchanged:

```text
H0 maximum violation: 0
H0 ddxi_c:             0.000401232234
H0 slip_c:             0.027082170421
KKT residual:          2.90e-14
R1 residual:           2.67e-14
branch split:          1.16e-10
scale convergence:     9.14e-11
```

Slip-common `+/-` at `1/0.5/0.25`, representative xi-common and slip-differential controls all pass in
the same contact regime. This validates the diagnostic algebra only; production does not consume this law.

## Stop decision

`R2 CANDIDATE FOR NEXT RE-AUTHORIZATION=NO`, `R2 AUTHORIZED=NO`, and `R2 IMPLEMENTED=NO`.
The next allowed action is production contact-response integration attribution, not implementation of the
diagnostic affine law.

Evidence: [formal-v1](evidence/automated/r2-stager-affine-reference-attribution-formal-v1/r2-stager-affine-reference-attribution.json)
and [fresh replay-v1](evidence/automated/r2-stager-affine-reference-attribution-replay-v1/summary.json).
