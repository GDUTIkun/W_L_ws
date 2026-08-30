# Phase 44 Addendum — Directional Authority Audit

Authoritative evidence: `evidence/automated/regime-authority-formal-v4/` and fresh
`regime-authority-replay-v4/`.

## Result

All 192 signed probes at each of `1.0/0.5/0.25 delta` retained the exact frozen regime signature,
including contact geom-pair/dimension topology and assembled inequality rows `12..103` lower/upper
near-active codes. The repaired classification is:

| Class | Count |
| --- | ---: |
| R44-S | 396 |
| R44-P | 84 |
| R44-O+ / R44-O- / R44-B | 0 / 0 / 0 |

The 480 rows are `snapshot x input channel x output family`; 380 pass the complete directional
convergence gate. At channel level, 78/96 plus and 80/96 minus derivatives are trusted; both sides
are trusted for 76/96 channels. Untrusted directions remain classified by regime/branch but are not
used to assemble formal matrices. The largest failed convergence error is `0.74897`; the gate is
`0.05`.

The 84 R44-P rows have branch relative difference `0.05329..1.73918`; R44-S is at most `0.04714`.
Thus the original central Jacobian failure is reproducible directional branch splitting within the
observable frozen regime, not replay noise and not a detected topology/active-row transition. It
must not be averaged.

## tick0 trusted authority

Both signed directions and the half/quarter checks pass at tick0.

- B/native common: QP wheel gains are approximately `(+0.999999,+0.999999)`; MuJoCo gains are
  `(-0.509244,-0.508347)`.
- C/xi common: QP xi gains are approximately `(+1.000000,+1.000000)`; MuJoCo xi gains are
  `(+0.948543,+0.948331)`. The associated native wheel gains change from QP about
  `(-2.55760,-2.56815)` to MuJoCo `(+1.32813,+1.33185)`.

These values confirm the original tick0 findings without symmetric averaging.

## Limits

The signature is complete for the frozen observable fields and assembled near-active rows, but
solver multipliers remain unavailable. The result does not claim smoothness below `0.25 delta` or
outside the audited snapshots.

