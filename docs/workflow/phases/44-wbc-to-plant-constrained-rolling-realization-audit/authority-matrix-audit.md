# Local Authority Matrix Audit

Evidence: [`authority-matrices.json`](evidence/automated/realization-audit-formal-v1/authority-matrices.json)

Frozen central probes use xi delta `0.01 m/s^2`, native delta `0.2 rad/s^2`, plus half-delta checks. At shared
tick0:

- B/native: QP wheel self gain is about `+1`; MuJoCo wheel gain is about `-0.509` (sign reversal).
- C/xi: QP xi self gain is about `+1`; MuJoCo xi self gain remains about `+0.948`, while the associated native
  wheel response changes from about `-2.56` predicted to `+1.33` actual.
- D exposes four-input coupling; its tick0 plant matrix is already poorly conditioned (`~7.42e3`).

Thus C can realize the corresponding plant xi observable while the native mode follows a different direction.
This directly explains the qualitative Phase43 paradox and is inconsistent with treating xi and wheel spin as one
rolling coordinate.

However DG44-06 fails: across all frozen late snapshots the maximum odd-symmetry error is `0.61634` and the
half-delta gain difference is `0.51337`, above the predeclared `0.05` bounds. Some late states are locally
nonsmooth/active-set or contact-regime sensitive. The matrices are reproducible (`fresh replay error=0`) but not a
single trustworthy linearization over the required snapshot set. Per the frozen classification rule this blocks a
final P44-E conclusion and forces P44-U.
