# R2 Contact-Law / Reduced-WBC Integration First-Mismatch Attribution

## Decision

Classification is `B-CONTACT-REACTION-REPRESENTATION-MISMATCH`. This round is attribution only;
production controller numerics did not change, no R2 repair is authorized or implemented, and Phase 46
remains `review / REWORK`.

The historical Stage-R failure was reproduced fresh before any witness analysis: H0 maximum violation
`0.0368511841794`, `ddxi_c=-6.86299498911`, `slip_c=-0.783202206426`, branch split
`3.12034206829`, and scale-convergence error `8.29578558891`. The source oracle also remained closed
(`qacc 4.34e-14`, contact-row force `3.11e-15`).

## First wrong integration relation

After applying the already-frozen, nonmaterial legal-equality conditioning, the actual H0 witness closes
full dynamics at `2.84e-14`, reduced dynamics at `2.13e-14`, corrected R1 at `2.73e-14`, and the native
constitutive law at `1.33e-15`. Constraint-row reaction and Cartesian point force give identical
generalized force.

The first material hard failure is:

```text
Aw_prod * W_actual = Qc0_prod + Qct_prod * tau_current
```

Its maximum residual is `4.83664403838`. The row/point generalized-force parity is `0`, but mapping the
rank-5 production aggregate wrench back through the Stage-R generalized-force operator has the same
`4.83664403838` residual. Each side's aggregate map has rank 5 and one point-force redistribution null
direction. Stage R therefore mixed a constraint-row constitutive outcome with an independent aggregate
wrench decision without preserving the exact row -> point -> aggregate representation and redistribution
semantics.

This is not closure double-count: equality rows participate in the eliminated full oracle but were not
added as an independent reduced hard relation. It is also not an optimizer root cause: the actual physical
witness is already hard-infeasible at the representation equation, so optimization/KKT attribution is not
entered.

The raw acceleration lift residual (`0.0516389674`, frozen-observable effect `0.0107606912`) and conditioned
rank-4 affine residual (`3.75698e-4`) are retained explicitly. They are the previously frozen nonmaterial
legal-equality response, not promoted into a new source.

## Stop decision

No corrected reduced law was derived, no Stage-R retry or directional probe was entered, and aggregate
wrench decision compatibility remains `INCOMPATIBLE` for this Stage-R law. The only allowed next action is
additional reduced-integration attribution; R2 candidate/authorization/implementation all remain `NO`.

Evidence: [formal-v2](evidence/automated/r2-contact-law-reduced-integration-attribution-formal-v2/r2-contact-law-reduced-integration-attribution.json),
[fresh replay-v2](evidence/automated/r2-contact-law-reduced-integration-attribution-replay-v2/summary.json),
and [historical Stage-R reproduction](evidence/automated/r2-stage-r-historical-reproduction-v1/summary.json).
