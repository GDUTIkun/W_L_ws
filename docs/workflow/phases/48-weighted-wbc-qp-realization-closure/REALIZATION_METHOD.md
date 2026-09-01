# Phase 48 — Realization Method and Semantic Contract

Status: `frozen by P48-T01`

## Authoritative Paths and Profiles

The public current runtime remains unique:

```text
RobotState → ControllerCore::stepWeightedWbc → WeightedWbcController::step
→ 42D WeightedWbcProblem → TorqueCommand → Adapter → MuJoCo
```

Phase48-A uses two frozen diagnostic H0 profiles only for regression:

| Role | Config / case | Controller profile | Contact law |
| --- | --- | --- | --- |
| baseline | `phase46_point_realizable_rolling_v1.json` / `R46P-H0` | `kPhase46PointRealizableRolling` | point-realizable aggregate image |
| primitive-R2 | `phase46_mujoco_contact_response_v1.json` / `R46M-H0` | `kPhase46MujocoContactResponse` | `fc=Dc(aref-Jc(N*nudot+cN))` hard rows |

They are not alternate current ROS runtimes. The frozen diagnostic selection is in
`phase35_workspace_attribution_loop.cpp`; current deployment remains
`current_weighted_wbc.launch.py`.

## Common Wrench Contract

All `W_ref`, `W_WBC` and reconstructed `W_MJ` values use:

```text
[L_Fx,L_Fy,L_Fz,L_Tx,L_Ty,L_Tz,R_Fx,R_Fy,R_Fz,R_Tx,R_Ty,R_Tz]
```

- frame: controller body FLU;
- reference point: corresponding wheel-body origin;
- actor/receiver: wheel follower wrench acting on leg/base;
- sign: force and moment in that actor/receiver direction;
- units: force N, moment N m.

### W_ref

- producer at H0: frozen reference stager/config; in the current Core path the same storage is filled
  from `currentNominalWeightedWbcConfig()` or the same-call NMPC override;
- storage: `WbcReference::interaction_wrench_flu[12]`;
- consumer: `WeightedWbcProblem::assemble` in the synchronous controller call;
- state ownership: the `RobotState` passed to that same `WeightedWbcController::step`.

There is no ROS wrench message and no second public reference route.

### W_WBC

The solver's scaled variable is decoded to the physical 42D solution:

```text
zphysical = [nudot(12), tau(6), W_L(6), W_R(6), slack(12)]
```

For both Phase48-A profiles the per-wheel decision wrench is post-projected by the frozen rank-5
point-force projector. The authoritative interaction wrench is then reconstructed as:

```text
W_WBC = A_interaction_acceleration * nudot
      + A_interaction_contact * W_projected
      + b_interaction
```

Physical projector: per-wheel corrected rank-5 projector. Post-solve reference transport: `NONE`.
Additional latent/null wrench use: `NONE`. `W_WBC` is not `W_ref`, a diagnostic closest-feasible
wrench, MuJoCo reaction or a slack-adjusted label.

### Tau

- physical decision block: `zphysical[12:18]`, N m;
- joint and actuator order:
  `[left_hip,left_knee,left_wheel,right_hip,right_knee,right_wheel]`;
- Core mapping: exact variable-scale decode; the controller rejects an over-limit solution instead of
  silently clamping it;
- `tau_raw_nm` and `TorqueCommand` are identical for an accepted result;
- Adapter resolves the same named actuator order, requires gear `1.0`, and writes
  `data->ctrl[actuator_id] = -TorqueCommand[index]`;
- additional motor/gear conversion: `NONE`.

### W_MJ

The authoritative diagnostic reconstruction is:

```text
MuJoCo active contact rows / efc_force
→ Cartesian contact-point forces
→ left/right grouping
→ per-wheel rank-5 production-reference aggregate
→ controller-body FLU wrench at wheel-body origin
```

The native constraint sign and contact frame are resolved row-by-row before aggregation; a numeric
direction match is not used as semantic evidence. `W_MJ` is diagnostic only and remains outside the
public ROS/Core feedback interface.

## Temporal and State Provenance

For fixed-H0 Phase48-A:

1. the native authority row is `record_kind=pre_command`, `control_tick=0`;
2. `W_ref`, QP assembly, `W_WBC` and `tau` share that single frozen RobotState snapshot;
3. the diagnostic applies MuJoCo `ctrl=-tau` at the same qpos/qvel and evaluates the constrained
   reaction without integration;
4. `W_MJ` is reconstructed from that reaction snapshot; it is neither a pre-command zero-torque
   reaction nor a post-step/next-state reaction;
5. no `state_next` is produced by this fixed-state gate.

Thus temporal/state provenance passes because ownership and ordering are explicit and reproducible,
not because all values are asserted to represent an identical continuous-time physical instant.

For the later dynamic runtime, the existing order is state extraction → synchronous controller solve →
command acceptance/write → five MuJoCo substeps. That later sampling contract is intentionally outside
P48-T01/T02.

## Slack and Normalization

The only 12D slack is interaction-wrench fidelity:

```text
rW = W_WBC - W_ref - signed_slack
```

No contact, rolling, base, orientation or leg task residual is called slack. Per-side normalization is:

```text
[Fx,Fy,Fz,Tx,Ty,Tz] / [50,50,50,2.5,2.5,2.5]
```

and both wheels share the same scales. `maximum_normalized_slack` is the maximum absolute normalized
component.

## Parity Gates

| Gate | Result | Evidence basis |
| --- | --- | --- |
| P48-A-PARITY-01 W_ref → QP input | PASS | source trace, frozen row and order |
| P48-A-PARITY-02 QP wrench → W_WBC | PASS | physical decode/project/reconstruction chain |
| P48-A-PARITY-03 tau → actuator | PASS | named order, gear invariant, explicit Adapter sign |
| P48-A-PARITY-04 MuJoCo reaction → W_MJ | PASS | row/contact/point/aggregate reconstruction chain |
| P48-A-PARITY-05 W_WBC ↔ W_MJ semantic space | PASS | common frame/origin/sign/order/unit contract; no equality requirement |
| P48-A-PARITY-06 temporal/state provenance | PASS | explicit fixed-state pre-command ownership |

## Frozen Source Anchors

- `ros_ws/src/wheel_leg_core/src/controller_core.cpp`
- `ros_ws/src/wheel_leg_core/src/weighted_wbc_problem.cpp`
- `ros_ws/src/wheel_leg_core/src/weighted_wbc_controller.cpp`
- `ros_ws/src/wheel_leg_mujoco/src/adapter.cpp`
- `ros_ws/src/wheel_leg_mujoco/src/phase35_workspace_attribution_loop.cpp`
- `tools/experiments/run_phase45_contact_consistent_rolling.py`
- `tools/experiments/run_phase46_wrench_slack_closure.py`

Any future conflict is a semantic regression and stops before P48-T03.

