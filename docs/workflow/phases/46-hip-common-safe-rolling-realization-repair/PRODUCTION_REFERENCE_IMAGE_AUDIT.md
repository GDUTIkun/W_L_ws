# Phase46 production-reference point-force image audit

## Verdict

`A-PRODUCTION-REFERENCE-IMAGE-CLOSED`.

Starting from the frozen actual two-point map, the unique production-reference map is
`Gp_prod = Tw Gp_point`, where `Tw` transports the wrench from the actual MuJoCo contact-center
reference to the production Model-B contact-center reference.  The corresponding twist transform
is `Tw^T`, preserving virtual work.

Both sides have rank 5.  The missing wrench directions in `[Fr, Fl, Fn, Mr, Ml, Mn]` order are:

- left: `[-1.80932e-4, 0, -1.16173e-4, 0, 0.999999977, 0]`;
- right: `[-2.55294e-4, 0, +6.06580e-5, 0, 0.999999966, 0]`.

For `Pg_prod = Gp_prod pinv(Gp_prod)`, symmetry is exact; idempotence is `<=1.11e-15`, range
containment `<=1.13e-15`, and projected-basis point-force reconstruction `<=1.37e-14`.
Transported full and reduced operator parity close to `1.67e-16` and `4.44e-15`; deterministic
virtual-work parity passes.

The current controller projector is not `Pg_prod`.  Its difference from the true projector is:

- left: spectral `1.21140e-4`, Frobenius `1.71317e-4`, max element `1.16173e-4`;
- right: spectral `1.19751e-4`, Frobenius `1.69354e-4`, max element `1.03252e-4`.

The largest projector-column difference is `Ml` on both sides.  Therefore the true
production-reference R1 image is now known, but R1 is not exactly closed in the current controller.
No corrected projector was implemented.

Formal evidence is
[formal-v1](evidence/automated/production-reference-image-audit-formal-v1/production-reference-image-audit.json)
and [fresh replay-v1](evidence/automated/production-reference-image-audit-replay-v1/summary.json),
with semantic error `0`.
