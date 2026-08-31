# Phase46 post-exact-R1 attribution

> Superseded scope note: the later full operator audit proves that this document's 7.5% mapping
> number mixed actual/production reference and reduction semantics.  It remains diagnostic only;
> see `WRENCH_GENERALIZED_FORCE_OPERATOR_AUDIT.md`.

## Verdict

`C-MAPPING-OR-REFERENCE-REGRESSION`.

Exact R1 remains closed and the before/after state and contact regime are identical.  The post-R1
audit stops before authorizing a contact-response repair because the same exact, point-realizable
wrench does not produce the same reduced generalized force through the production aggregate-wrench
map and the reconstructed point-contact map.

## Frozen comparison

The comparison is Phase45 compatible-H0 versus the exact `P_G` candidate at tick0.  The maximum
differences in `q`, `qdot`, mass, bias, reduction, reduction bias, contact Jacobian/bias, contact map,
xi map, and xi bias are at most `3.38e-17`.  Both cases retain two dimension-3 contacts per side and
the same topology, positive normal load, penetration, and interior-friction signature.

The exact candidate changes QP ddxi by `+9.87291e-8 / +5.30529e-6 m/s2`, while actual MuJoCo ddxi
changes by `+0.0379952 / -0.0752634 m/s2`.  On the frozen plant:

- actuator/free contribution: `+0.0378838 / -0.0432577 m/s2`;
- QP contact prediction: `-0.0362691 / +0.0447233 m/s2`;
- actual contact response: `+0.000232924 / -0.0328646 m/s2`;
- actual-minus-QP contact gap: `+0.0365020 / -0.0775879 m/s2`;
- gap/observed norm ratio: `1.01702`.

Actual ddxi, causal ddxi, generalized-force, and actuator-sum closures are respectively
`4.84e-14`, `4.85e-14`, `1.42e-14`, and `2.08e-17`.

## First material mismatch

For the exact post-R1 wrench, `G_p G_p^dagger w = w` closes to `1.42e-14` left and `7.11e-15`
right.  Point-force realizability therefore passes.  However, mapping that same wrench through the
two frozen paths gives reduced-generalized-force maximum errors of `2.40757` left and `2.48120`
right, relative errors `0.07511 / 0.07503`.  This is material, so the mandatory same-wrench mapping
parity gate fails and classification C precedes any contact-response-law conclusion.

The solver regime is stable and its rolling reaction opposes the torque-induced free motion at all
four points.  It is not the first mismatch.  The historical approximate-candidate label
`R2-CONTACT_RESPONSE_MISMATCH_AFTER_R1` remains historical/non-authoritative; the exact-R1 form is
not authorized.

Formal evidence is
[post-exact-r1-attribution-formal-v4](evidence/automated/post-exact-r1-attribution-formal-v4/post-exact-r1-attribution.json),
with [fresh replay-v4](evidence/automated/post-exact-r1-attribution-replay-v4/summary.json) semantic
error `0`.  No repair, tuning, trajectory, AUTH, REAL, SHORT, 10 s, or NMPC run was performed.
