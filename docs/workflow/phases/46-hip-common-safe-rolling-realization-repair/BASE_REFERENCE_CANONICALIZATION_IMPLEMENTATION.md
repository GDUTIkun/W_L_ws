# Base Reference Canonicalization Implementation

## Result

Classification: `A-DIAGNOSTIC-BASE-REFERENCE-CANONICALIZATION-IMPLEMENTED`.

The authorized candidate is implemented only in the Phase46 cross-model diagnostic boundary,
before comparisons of `M/h/Q/J/N/qacc/observable`. Production controller, reduced QP, state
semantics, model parameters, contacts and equality constraints are unchanged.

`DG46RC-COMP` passes configuration/twist/force round trips, virtual-power/full-dynamics
covariance, mass, Jacobian and reduction covariance, and observable invariance. The maximum
reported covariance residual is `1.78e-15`. Fresh controller CSV regression, excluding runtime
timing telemetry, is exactly zero; R1 remains closed and the production reduced QP remains valid.

## Common4 re-attribution

The frozen slip-common common4 gap changes from `-0.388382869511` to `-0.008598876446`, removing
`-0.379783993065` or `97.7859795%`. The reference-semantic mismatch is therefore closed and the old
precontact physical-mismatch interpretation is superseded. The residual secondary inertial-family
gap is nonmaterial.

The physical-channel re-decomposition closes without double counting. Legal equality remains
nonmaterial, while contact response (`-0.748977633253`) and the independent MuJoCo-only closure
mechanism remain material. Contact is consequently not the unique material remaining mismatch;
R2 is neither a candidate nor authorized. The only next allowed action is closure-model
attribution.

Evidence: [formal-v3](evidence/automated/base-reference-canonicalization-implementation-formal-v3/base-reference-canonicalization-implementation.json)
and [fresh replay-v1](evidence/automated/base-reference-canonicalization-implementation-replay-v1/summary.json),
with semantic replay error `0`.
