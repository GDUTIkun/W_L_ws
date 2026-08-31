# R2 Contact-Reaction Representation Commuting-Diagram Attribution

## Decision

Classification is `A-AGGREGATE-DYNAMICS-SUFFICIENT-STAGER-AFFINE-MAP-MISMATCH`.
No production controller, QP, representation, contact law, gain, weight, or solver setting changed.
Phase 46 remains `review / REWORK`; R2 remains unauthorized and unimplemented.

The previous attribution correctly localized the historical failure to representation integration, but its
claim that aggregate-wrench decision semantics were incompatible was too strong. Fresh reconstruction
shows that the `4.83664403838` residual is not point-force quotient loss. It is a generalized-force
coordinate mismatch: `Aw*W` remained in MuJoCo base-body-origin coordinates (`M`) while
`Qc0+Qct*tau` had been transformed to the production base-control reference (`P`).

## Commuting diagram

Fresh actual geometry, contact points, point Jacobians, production-reference transport, `Gp`, and `Aw`
give:

```text
max |Aw*Gp - Jp^T| full:     1.66533453694e-16
max |Aw*Gp - Jp^T| reduced:  1.66533453694e-16
virtual-work residual:       5.76571355685e-16

E1 row -> point:             4.88498130835e-15
E2 point -> aggregate:       8.88178419700e-16
E3 corrected P -> P:         1.24927845846e-13
historical mixed M -> P E3:  4.83664403838
```

Thus row reaction, Cartesian point force, and production aggregate wrench commute for generalized
dynamics. The historical error lies on the aggregate-to-Stage-R edge only because the force-dual
base-reference canonicalization was omitted there.

The historical residual decomposes into a `1.52219637586` affine-offset frame contribution and a
`3.31444766252` slope-times-current-torque frame contribution; their signed sum recreates the
`4.83664403838` dominant component. Applying the same `M -> P` force transform to both closes H0 to
`1.25e-13`.

## Sufficiency and nullspace

Each `Gp` has rank 5 and nullity 1, but `Jp^T n_p` is numerical. Actual H0 null amplitudes are
`eta_L=-6.08e-16`, `eta_R=7.85e-16`; all frozen directional witnesses remain numerical. Structural
point-force redistribution therefore does not supply an independently material R2 mechanism in this
audit. Aggregate wrench is dynamics-sufficient, and no point-force/eta/lambda decision-variable redesign
is justified.

All slip-common `+/-` scales `1/0.5/0.25`, representative xi-common `+/-`, and slip-differential `+/-`
replays close E1/E2 and corrected E3 at machine scale. No controller was re-solved with a new law.

## Stop decision

`R2 REPRESENTATION CHANGE REQUIRED=NO` and
`R2 REPRESENTATION CANDIDATE FOR NEXT ROUND=NO`. The next allowed action is Stage-R affine
reaction-map attribution, preserving aggregate wrench and explicitly auditing the `M/P` force-dual
canonicalization boundary. This round does not authorize that implementation.

Evidence: [formal-v2](evidence/automated/r2-contact-reaction-commuting-diagram-attribution-formal-v2/r2-contact-reaction-commuting-diagram-attribution.json),
[fresh replay-v2](evidence/automated/r2-contact-reaction-commuting-diagram-attribution-replay-v2/summary.json),
and [operator replay-v3](evidence/automated/wrench-generalized-force-operator-audit-replay-v3/summary.json).
