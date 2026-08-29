# Wheel interaction-wrench contract and affine reconstruction

Date: 2026-08-29

Decision: `DG27-02 PASS`

## Physical quantity

For each side, the requested and realized quantity is the total wrench
exerted by the wheel follower on the leg/base, about the wheel-body origin,
expressed in controller body/FLU. Side order is left then right and component
order is `[Fx,Fy,Fz,Tx,Ty,Tz]` in N/Nm.

This exactly matches historical contract `09-01-G1` in
`wheel_interface_wrench_contract.m`: follower actor, leg/base receiver,
`FollowerOnBase` sign and coincident wheel-centre moment origin. It is not the
external ground-contact resultant about the base-control point.

## Newton--Euler derivation

Let `C` be the current contact-wrench origin, `O` the wheel-body origin,
`r_OC=p_C-p_O`, `r_OG=p_G-p_O`, and let the wheel inertia be about its COM `G`.
The WBC contact variable is the wrench exerted by ground on wheel at `C`.
With all world quantities evaluated at fixed `q,nu`:

```text
F_ng = m_w (a_G - g)
M_ng^O = I_G alpha + omega x (I_G omega) + r_OG x F_ng
W_wheel_on_parent^O = transport(C -> O, W_contact_on_wheel^C)
                       - [F_ng; M_ng^O]
```

The minus sign is action/reaction: the parent-on-wheel wrench closes the wheel
free body; the required output is wheel-on-parent. A stationary unsupported
wheel therefore has a negative FLU `Fz` bias, matching the historical
`gravityForce - mass*biasV` term.

At fixed `q,nu`, wheel COM and angular accelerations are affine in reduced
`nudot`, while the contact transport is linear. Therefore:

```text
W_I = A_nudot(q) nudot + A_contact(q) w_C + b_I(q,nu)
```

The contact lever arm `r_OC` occurs exactly once in `A_contact`. The older
`wrench_flu_map`, whose lever arm is `p_C-p_base_control`, is not used.

## Code boundary

`NominalWbcModel::Result` now exports, per side,
`interaction_acceleration_map`, `interaction_contact_map` and
`interaction_bias`. These are additive live-model outputs; the existing
reduced dynamics, contact map and Phase 23 external-wrench map are unchanged.

## Oracle and algebra results

The versioned independent MuJoCo generator reconstructs the map from compiled
body/geom poses, body mass/COM/inertia, MuJoCo point/angular Jacobians and
central-difference bias acceleration. It does not call the C++ implementation.
Four equilibrium/3D/chart-boundary samples are stored in
`automated/wheel-interaction-oracle-v1` and consumed by the C++ component test.

Maximum C++ versus oracle errors are:

| Quantity | Maximum error | Gate |
| --- | ---: | ---: |
| wheel position/velocity | `8.68e-17` | `2e-9` |
| acceleration map | `5.56e-17` | `2e-8` |
| contact transport map | `4.45e-15` | `2e-10` |
| velocity-dependent bias | `1.25e-8` | `2e-5` |

The component corpus additionally passes both-sign full-component pulses,
left/right equilibrium symmetry, 6D transport round-trip, wrench/twist
virtual-work invariance, affine superposition, stationary-wheel action/reaction
sign and exact signed-slack identity `W_real-W_request-s=0`. Thus the map is
approved as the shared Phase 27 NMPC/WBC authority. T07 must still prove its
actual QP row/objective assembly against this identity.
