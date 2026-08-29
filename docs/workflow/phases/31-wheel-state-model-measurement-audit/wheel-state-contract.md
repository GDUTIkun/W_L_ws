# Phase 31 wheel-state contract

## Authoritative chain

`Adapter` does not publish a reduced wheel state. The live chain is:

```text
MuJoCo qpos/qvel
  → Adapter RobotState (base_control_frame pose/twist + six active joints)
  → NominalWbcModel passive closed-chain reconstruction
  → xi_L, xi_R, dxi_L, dxi_R
  → WheelAwareNmpcModel states 12..15
```

For side `s`, with wheel-body origin `O_s`, base-control site `B`, and
`R = R_N_from_B`:

```text
xi_s  = e_x^T R^T (p_Os^N - p_B^N)
dxi_s = e_x^T [R^T(v_Os^N - v_B^N) - omega_B^B × r_Os/B^B]
```

- origin: `left_wheel_body` / `right_wheel_body` body origin;
- reference point: site `base_control_frame`;
- frame and positive direction: controller body/FLU, `+x` forward;
- `xi` is geometric wheel-center displacement relative to the base-control point;
- it is not wheel spin angle, rolling arc length, or contact-patch displacement;
- `dxi` is the analytic time derivative in the rotating base frame, not a sampled difference;
- common/differential order is `c=(L+R)/2`, `delta=(R-L)/2`.

## Evidence

The Phase30 20 ms authority mismatch reproduced exactly. Direct MuJoCo geometry and analytic
relative velocity then passed the frozen contract:

| Gate | Maximum | Limit | Result |
| --- | ---: | ---: | --- |
| control/plant alignment | `5.42e-20` | `1e-10` | PASS |
| core `xi` vs geometry | `1.53e-16 m` | `5e-5 m` | PASS |
| core `dxi` vs analytic kinematics | `1.98e-16 m/s` | `2e-3 m/s` | PASS |
| 2 ms centered geometry FD vs analytic `dxi` | `1.525e-3 m/s` | `2e-3 m/s` | PASS |

`wheel-state-contract-v1` and fresh replay `v2` have byte-identical semantic outputs. Therefore
`P31-A` and `P31-B` are excluded on the authority corpus; no Adapter or measurement change is
authorized.

