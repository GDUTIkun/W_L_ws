# Static geometry audit

Authority: `evidence/automated/wheel-phase-validity-formal-v2`; fresh replay:
`evidence/automated/wheel-phase-validity-replay-v1`.

Across 75 contact-on and 75 contact-off fixed-state samples:

- wheel-center and wheel-origin change from phase alone: `0 m`;
- xi/zeta and wheel-origin Jacobians remain phase invariant;
- the inherited Phase35 frozen state has absolute closure residual
  `1.6348568008428543e-4 m`, but its variation over the complete phase sweep is exactly `0 m`;
- all values are finite;
- raw contact centroid changes by as much as `5.462666654638799e-2 m` and contact count changes
  between phases.

The absolute closure residual is reported, not relabeled as zero. This Phase preserves the accepted
Phase35 state and tests phase-induced degradation; wheel phase produces none. The large contact
variation occurs while wheel origin/leg geometry is identical, directly locating it below the
rigid wheel-origin layer at the rotating collision mesh/contact manifold.

DG36-01: **PASS** (audit complete and phase effect identified).
