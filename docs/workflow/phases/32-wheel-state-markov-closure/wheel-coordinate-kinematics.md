# Phase 32 wheel-coordinate kinematics

Let `r_B=R_NB^T(p_w-p_b)`, `omega_B=R_NB^T omega_N`, and
`v_rel,B=R_NB^T(v_w-v_b)-omega_B×r_B`. Then

```text
xi   = e_x^T r_B
dxi  = e_x^T v_rel,B
ddxi = e_x^T [R_NB^T(a_w-a_b)
               - 2 omega_B × v_rel,B
               - alpha_B × r_B
               - omega_B × (omega_B × r_B)]
```

The same equations with `e_z` define `zeta/dzeta/ddzeta`. The acceleration oracle does not read a
logged acceleration approximation. It runs floating-base `mj_forward`, saves the resulting `qacc`,
integrates `qpos` by `qvel*epsilon`, changes `qvel` by `qacc*epsilon`, and centered-differences the
independent analytic moving-frame velocity. Repeating at `epsilon=1e-6` and `5e-7 s` gives a maximum
disagreement of `3.53e-10 m/s²`, far below `0.05 m/s²`.

Position and velocity parity inherited from Phase31 remains valid: direct `xi` error is
`1.53e-16 m`, Adapter/analytic `dxi` error `1.98e-16 m/s`, and trajectory-FD/analytic `dxi` error
`0.001525 m/s`. Wheel spin changes neither wheel-body-origin position nor its translational velocity;
it changes material-point rolling velocity and the MuJoCo contact solve.
