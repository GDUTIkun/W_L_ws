# Wheel-aware 16-state model contract

Date: 2026-08-29

Decision: `DG27-03 PASS`

## State, chart and input

The Phase 27 state is

```text
x = [p_B^N(3), r(3), v_B^N(3), omega_B^N(3),
     xi_L, xi_R, dxi_L, dxi_R]
```

`B` is `base_control_frame`. The physical attitude is
`R_N_from_B=Exp(r) R_ref`; `r` is the world-axis shortest-arc relative
rotation vector and `omega_B^N` is physical world angular velocity. This
reuses the Phase 23 canonical chart and rejects the historical Euler-rate
state. One `R_ref` is fixed for an entire 20 ms solve and 0.4 s horizon. The
runtime must choose a yaw-aligned desired anchor on every solve, so the T2
`0.08 rad/s` horizon excursion is about `0.032 rad`, not an indefinitely
accumulating absolute yaw. `||r||<=0.35 rad` remains mandatory.

The input is left then right wheel-on-body interaction wrench at the
wheel-body origin, in instantaneous body FLU, with each side ordered
`[Fx,Fy,Fz,Tx,Ty,Tz]`. The moment about `B` is reconstructed once as
`T_i + r_BO_i x F_i`; contact-centred wrench is never read by this model.

Model-layer validity additionally requires the Phase 15 side workspaces
`xi_L in [-0.3303432354,0.1678677251] m` and
`xi_R in [-0.3321211483,0.1659029424] m`. Other state channels must be finite;
the model does not invent global position/twist bounds. Numeric OCP performance
bounds and costs remain DG27-05 and cannot change this physical validity gate.

The reference has the same order. With `R_ref` equal to desired attitude its
orientation reference is zero; common planner values are copied to both
`xi` and `dxi` entries and differential reference remains zero. The model
equilibrium input is stored in the v2 oracle summary and is an exact upper-body
static balance with zero `Fx/Ty`, so Eq. (12) also has zero derivative.

## Current nominal parameters and equations

MuJoCo inertials were aggregated at the Phase 23 equilibrium about `B` after
explicitly excluding `left_wheel_body` and `right_wheel_body`:

```text
m_b = 5.748200000000001 kg
c_B = [-0.011186360321930223,
        0.00010351112192572815,
       -0.05007382006473043] m
I_COM,B =
  [ 0.14032539391425894  -0.00027482615417932 -0.00553012807897188
   -0.00027482615417932   0.07534641495796530  0.00019749711948993
   -0.00553012807897188   0.00019749711948993  0.09406865781352407 ] kg m^2
```

Each current wheel has `m_w=0.3431 kg`, nominal rolling radius `rho=0.05 m`,
and axle inertia `I_w=0.000423737590895418 kg m^2`, the symmetric mean of the
two compiled bodies projected onto their actual joint axes. Thus
`D_w=m_w rho+I_w/rho=0.02562975181790836 kg m`. Historical `7 kg`, `0.08 m`
and `0.00112 kg m^2` values are not used.

Rigid-body translation/rotation use the frozen upper composite, full
non-diagonal inertia and the current wheel-origin lateral/vertical lever arms.
Wheel-relative acceleration is exactly

```text
ddxi_i = -(Fx_L+Fx_R)/m_b - (rho Fx_i + Ty_i)/D_w.
```

The continuous chart rate is `rdot=J_l(r)^-1 omega`. The discrete map is a
20 ms ZOH with two fixed 10 ms RK4 substeps. This does not change the NMPC
sample period; it replaces the failed one-substep discretization described
below.

## Independent oracle and component results

The MuJoCo/NumPy/SciPy generator independently aggregates compiled body
inertials, projects wheel inertia along the actual axle, forms the equations,
integrates DOP853 and central-differences both continuous and discrete maps.
Its six samples cover equilibrium, common, differential, left-only,
right-only and non-zero-yaw anchored 3D states.

- `wheel-aware-model-oracle-v1` is retained as FAIL: a single 20 ms RK4 step
  gave DOP853 error `2.7864e-7` and step-doubling error `2.6107e-7`, above the
  pre-frozen `3e-8` gates. No threshold was widened.
- Superseding `wheel-aware-model-oracle-v2` uses two 10 ms RK4 substeps. All
  ten gates PASS. Maximum DOP853/step-doubling errors are `1.7571e-8` and
  `1.6468e-8`; continuous/discrete FD step-stability is `1.0659e-8` and
  `2.7756e-10`; Eq. (12) mode identity and repeat errors are exactly zero;
  equilibrium derivative is `5.4521e-16`.
- C++ forward AutoDiff parity over the same corpus reports maximum continuous,
  next, continuous-Jacobian and discrete-Jacobian errors
  `2.1317e-14`, `3.8858e-16`, `7.2271e-9`, `1.0385e-10`.
- Release Core build passed after one retained implementation-only compile
  failure caused by missing Eigen fixed-size RK4 stage temporaries. Restoring
  those temporaries fixed type deduction without changing equations.
- Core suite after the fix: `29 tests, 0 errors, 0 failures`.

Dependency probe before stable v2 output recorded MuJoCo `3.7.0`, NumPy
`2.2.6`, SciPy `1.15.3`, CasADi `3.7.2` and the project acados template under
`/home/t/opt/acados`; the generator passed `py_compile`.
