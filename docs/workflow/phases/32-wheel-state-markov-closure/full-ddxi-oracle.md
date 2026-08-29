# Phase 32 floating-base full-ddxi oracle

## Authority correction

Phase31 semantic summaries reproduce all six frozen facts, but its controlled input-response script
loaded the XML default `base_weld` and never disabled it. Production `weighted_wbc_loop` explicitly
runs `AdapterConfig::floating_base=true`. At T0 tick 54 the welded replay produces approximately
`[50.11,51.28] m/s²`, while the corrected floating-base replay produces
`[1.345,1.518] m/s²`. Thus Phase31 measurement and trajectory-FD evidence remains authority, but its
controlled MuJoCo gain/effective-inertia attribution is superseded for dynamics.

Phase32 sets `eq_active[base_weld]=0` before every `mj_forward`, applies canonical torque with the
Adapter sign, clears external wrench, requires finite `q/v/qacc`, and records all named wheel-floor
contacts, dimensions, penetration, constraint velocity and contact-frame force. All formal C1–C3
states retain positive normal load on both sides.

## Evidence

- C3 wheel rate: corrected-label authority `markov-closure-v3`, byte-identical replay `v4`
  (`v1/v2` retain the earlier ambiguous physical-wrench label);
- C1/C2 first scale: `leg-nullspace-v1` (inconclusive because C1 crossed patch switches);
- C1/C2 authority: corrected-label `leg-nullspace-v4`, byte-identical replay `v5`
  (`v2/v3` have identical numeric evidence);
- projection ranks: `projection-rank-v1`, byte-identical replay `v2`;
- mesh phase: corrected-label `wheel-angle-hybrid-v3`, byte-identical replay `v4`
  (`v1/v2` have identical numeric evidence).

The maximum two-epsilon oracle disagreement is `3.53e-10 m/s²`. The evidence directories refuse
overwrite and their manifests hash the method, scene, source, executable and raw authority logs.
