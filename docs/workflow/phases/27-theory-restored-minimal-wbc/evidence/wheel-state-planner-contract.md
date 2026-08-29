# Wheel-state and planner contract

Date: 2026-08-29

Decision: `DG27-01 PASS`

## State map

Side order is left, right. For wheel-body origin `O_s`, canonical base-control
point `B`, and `R = R_N_from_B`:

```text
xi_s  = e_x^T R^T (p_Os^N - p_B^N)
dxi_s = e_x^T {R^T(v_Os^N-v_B^N)
                 - (R^T omega_B^N) x [R^T(p_Os^N-p_B^N)]}
xi_c      = (xi_L + xi_R)/2
xi_delta  = (xi_R - xi_L)/2
dxi_c     = (dxi_L + dxi_R)/2
dxi_delta = (dxi_R - dxi_L)/2
```

`xi` is a forward geometric displacement in controller body/FLU, not wheel
spin angle. The point is the compiled wheel-body origin used by MuJoCo
`data.xpos`, which is the axle/wheel centre in the current model. No contact
point or wheel-geom centre is substituted.

The C++ `NominalWbcModel` exports the four per-side values additively. At the
equilibrium golden it returns
`xi_L=-0.009573649495650122 m`,
`xi_R=-0.012740695843911437 m`, and zero speeds, matching the independent
Phase 23 MuJoCo `state-oracle-v2` within `2e-10 m`.

## Workspace

The Phase 15/23 enumerated side envelopes are:

```text
left  [-0.3303432354,  0.1678677251] m
right [-0.3321211483,  0.1659029424] m
```

The common planner uses their conservative intersection
`[-0.3303432354, 0.1659029424] m`. This does not replace the original joint
workspace gate. Differential state is measured/predicted but its reference is
zero; there is no hidden differential low-level task in Minimal WBC.

## Governor

The version-1 planner is the deterministic bounded second-order governor
re-derived from the Simulink oracle:

```text
omega = 2 pi 2 Hz
a = clamp(omega^2(xi_target-xi_ref)-2(1)omega dxi_ref, +/-0.5 m/s^2)
dxi_ref+ = clamp(dxi_ref + dt a, +/-0.15 m/s)
xi_ref+ = clamp(xi_ref + dt dxi_ref+, workspace)
```

If the position clamp activates, velocity is set to zero and reported
acceleration is the realized velocity difference divided by `dt`; that
boundary-stop value may exceed the free-motion acceleration bound and is
reported rather than hidden. The target is clamped before the update.

Reset is bumpless: it accepts only finite measured common position/velocity
already inside the above bounds and copies them exactly with zero reported
acceleration. Invalid or out-of-contract reset leaves the planner
uninitialized. Invalid step input holds the current planner state; the runtime
integration must treat its validation failure as fail-zero/latch rather than a
last-valid controller fallback.

## Validation

- Independent MuJoCo oracle: Phase 23 `state-oracle-v2`, all nine gates PASS,
  maximum wheel position/rate finite-difference error `1.38e-10`.
- New C++ parity assertions use its equilibrium golden and explicitly audit
  left/right, common/differential signs.
- `test_wheel_position_planner` covers the exact first step, saturation,
  workspace stop, deterministic replay, reset, invalid config and invalid
  input.
- Release Core suite after the change: 27 tests, zero errors/failures.
