# Phase 32 full-state to reduced-state contract

## Production projection

The Phase27 state is

```text
x16 = [p_N(3), r_ref(3), v_N(3), omega_N(3), xi_L, xi_R, dxi_L, dxi_R]
```

where `p_N` and `v_N` are the `base_control_frame` site position and velocity,
`r_ref = Log(R_NB R_ref^T)`, and `omega_N` is the base angular velocity in world axes. For wheel
body origin `O_w`, base control origin `O_b`, and `R_NB`:

```text
r_B = R_NB^T (p_N(O_w) - p_N(O_b))
xi   = e_x^T r_B
dxi  = e_x^T [R_NB^T(v_N(O_w)-v_N(O_b)) - omega_B × r_B]
```

The live Controller fills indices `0:12` from `RobotState`, obtains `xi/dxi` from
`NominalWbcModel`, and writes them to indices `12:16`. The input remains the requested left/right
wheel-on-body internal wrench about each wheel-body origin, in body FLU.

## Full MuJoCo and RobotState fields

The nominal MuJoCo state has `nq=17`, `nv=16`: a free joint (`7/6`) followed by right
hip/knee/wheel/connect1/connect2 and left hip/knee/wheel/connect1/connect2 hinges. `RobotState`
retains the free-base state and the six active joints in canonical order
`[L hip,L knee,L wheel,R hip,R knee,R wheel]`; Adapter signs are canonical `q=-q_mj+offset` and
`dq=-v_mj`.

The projection discards:

- both active hip/knee coordinates and velocities except for their scalar effect on `xi/dxi`;
- all four passive closure coordinates and velocities;
- both wheel angles and wheel spin rates;
- wheel-origin relative vertical/lateral coordinates and velocities;
- soft-contact penetration, contact-patch identity, slip and contact force (algebraic MuJoCo
  derived fields, not independent integrator state).

The discarded quantities remain available at runtime either directly in `RobotState` (active joint
and wheel encoder fields) or through the already validated closed-chain geometry reconstruction.
The Phase32 evidence proves that several are dynamically material, so this projection is not
Markov-closed for the current plant.

## Minimum evidence-backed augmentation

For each side define

```text
zeta  = e_z^T r_B
dzeta = e_z^T [R_NB^T(v_N(O_w)-v_N(O_b)) - omega_B × r_B]
theta_w, omega_w = canonical wheel encoder angle and rate
```

The smallest currently evidenced superset is therefore
`x24=[x16,zeta_L,zeta_R,dzeta_L,dzeta_R,theta_wL,theta_wR,omega_wL,omega_wR]`.
This is a **necessary-state contract, not a sufficiency claim**. `zeta/dzeta` distinguish the C1/C2
leg/contact-compliance null directions; `theta_w/omega_w` distinguish mesh contact-patch phase and
rolling slip. A production schema or acados artifact is not authorized.
