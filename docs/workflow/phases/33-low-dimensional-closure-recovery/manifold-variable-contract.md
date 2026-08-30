# Manifold Variable Contract

For side `i` (left, right), let `p_c` be the canonical base-control point, `p_wi` the wheel-body
origin and `R_NB` the body-to-world rotation. In controller body/FLU:

```text
r_i^B   = R_BN (p_wi - p_c)
zeta_i  = e_z^T r_i^B                 (+z is up)
dzeta_i = e_z^T [R_BN(v_wi-v_c) - omega_B x r_i^B]
```

The frozen equilibrium source is
`test/data/phase21_weighted_wbc_problem_golden_v2.txt`, case `workspace_equilibrium`:

```text
zeta_ref_left  = -0.26587051502608749 m
zeta_ref_right = -0.26574406892872393 m
dzeta_ref      = [0, 0] m/s
```

These are geometric/contact-manifold references. They are constant and do not depend on NMPC
tracking error.

At fixed state, the reduced WBC acceleration gives

```text
ddzeta_i = A_zeta_i * nudot + b_zeta_i
```

where the wheel-origin world acceleration uses its tree Jacobian and closure reduction; base-control
linear acceleration is `nudot[0:3]`; body angular acceleration contributes
`skew(r_i^B) R_BN nudot[3:6]`; moving-frame Coriolis/centripetal and passive-closure acceleration form
the bias. A central derivative of independently reevaluated `dzeta` matched this expression to
`1.7181e-12 m/s^2` on a moving-leg state.

The new two rows are not aliases for the existing six contact rows: the minimum relative residual
after projecting a zeta row onto the contact-row span was `0.89693`.
