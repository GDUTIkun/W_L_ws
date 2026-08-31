# Phase46 wrench/generalized-force operator audit

## Verdict

`C-REFERENCE-POINT-MISMATCH`.

The production aggregate-wrench map and the independently rebuilt actual two-point operator use
different wrench/twist reference points.  Without transport, both full and reduced identities fail.
With the standard dual wrench/twist transport, both identities recover to machine precision.

## Operator result

For the raw identity `Aw Gp - Jp^T`, the full-coordinate residuals are:

- left: spectral `4.18341e-4`, Frobenius `5.11829e-4`, max element `1.69721e-4`, relative
  Frobenius `1.92481e-4`;
- right: spectral `3.38708e-4`, Frobenius `3.78687e-4`, max element `1.03252e-4`, relative
  Frobenius `1.42386e-4`.

The reduced-coordinate residuals are numerically the same scale: max element `1.69721e-4` left
and `1.03252e-4` right.  The dominant projected basis column is left `Fn`; the dominant DOF block
is left base rotation (Frobenius `4.17037e-4`).

The actual-minus-production reference offsets in contact coordinates are
`[-1.16173e-4,-1.69721e-4,-3.43335e-5] m` left and
`[+6.06580e-5,0,+1.03252e-4] m` right.  Transporting the wrench from the actual reference to the
production reference gives full residuals `<=1.67e-16` and reduced residuals `<=4.44e-15`.
Eight deterministic virtual-work vectors per side reproduce the raw mismatch and close after the
dual transport.

## Authority reconciliation

The old same-wrench audit remains valid only in its narrower scope.  It compared one directional
hip-common scalar after sending the same numeric wrench components through each model's own
reference/map.  Its `-3.51221e-5` scalar mapping term and `2.42575e-6` fraction did not test the
complete six-column identity and could not establish reference equality.

The later 7.5% result is superseded as an attribution-script quantity error: it combined the
production reduced map with an actual-reference point operator without the required transport and
also mixed production and plant reduction semantics.  The physical raw operator discrepancy is
about `1.4e-4--1.9e-4` relative and is fully explained by reference transport, not 7.5%.

The consequence is that exact R1 is not closed at the production wrench reference: the existing
projector differs from the correctly transported production-reference point-image projector by
`1.16173e-4` left and `1.03252e-4` right.  R2 remains unauthorized.  No repair or runtime experiment
was performed.

Formal evidence is
[operator formal-v2](evidence/automated/wrench-generalized-force-operator-audit-formal-v2/wrench-generalized-force-operator-audit.json)
and [fresh replay-v2](evidence/automated/wrench-generalized-force-operator-audit-replay-v2/summary.json),
with semantic error `0`.
