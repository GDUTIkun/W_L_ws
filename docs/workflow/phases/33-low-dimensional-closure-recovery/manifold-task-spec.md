# Minimal WBC Zeta-Manifold Task

The diagnostic enum `kPhase33ZetaManifold` reuses Phase27 Minimal interaction-wrench semantics and
adds only

```text
residual_i = A_zeta_i * nudot + b_zeta_i - ddzeta_des_i
scale      = 1 m/s^2
```

to H/g. For later closed-loop candidates only,
`ddzeta_des=-kp(zeta-zeta_ref)-kd*dzeta`; no gain was selected or run in this Phase because the
gain-free gate did not pass.

Component evidence proves:

- QP remains 42 variables and 104 hard rows with 12 dynamics equalities;
- A/lower/upper are exactly identical to Phase27 Minimal;
- Phase27 Minimal and Nominal code paths retain their existing objectives and wrench semantics;
- base-X, height, orientation and leg references remain ignored by the Phase33 profile;
- a non-finite zeta acceleration reference fails closed;
- equilibrium direct authority was `0.998789` self gain, `0.000923` cross/self and `0.9535%`
  centered algebraic-wrench change;
- `wheel_leg_core` Release build and all 16 tests pass.

The profile is not selected by `ControllerCore`; production therefore remains Phase27.
