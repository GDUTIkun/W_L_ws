# Large-angle numerical precision audit

Engineering horizon `|k| <= 1e6` remains clean; no recenter threshold is required there. The first
diagnostic material error occurs at `k=-5e7`, left-only: rotation representation normalized error
`3.061532696801983e-8`. At that state dynamic error is only `2.12496686913255e-13 m/s²`, contact
topology remains exact and all quantities remain finite. This is expected argument-reduction
precision growth, not a discontinuity or a ±1 rad boundary.

The `1e-6 rad` central-difference step remains representable throughout the frozen diagnostic
corpus, including ±5e8 revolutions. `large-angle-numerics.json` records ULP, step/ULP and derivative
error for every bilateral state; no NaN/Inf or catastrophic contact/solver transition was observed
in the offline sweep.

Conclusion: R0 is numerically healthy over the declared engineering lifetime. If a future product
can accumulate tens of millions of revolutions without reset and needs tighter orientation phase,
R2 recentering is a justified mitigation near that independently selected scale. It does not
justify the historical ±1 rad gate.
