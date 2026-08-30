# Wheel absolute-angle consumer audit

DG40-00: **PASS**. CBM generation `2026-08-29T06:47:42Z` located live code; metadata-changed
WBC files and excluded firmware/config/docs were verified directly. No unresolved live consumer
boundary remains. “Requires unwrapped” below means accumulated travel semantics, not physical
orientation.

| Class | Live consumer | Actual semantics | Wrap / revolution consequence |
| --- | --- | --- | --- |
| A rigid-body kinematics | MuJoCo hinge and `NominalWbcModel::forwardKinematics` | hinge orientation / `AngleAxis(q)`; periodic | no revolution-count dependence; q and q+2π are physical equivalents |
| B inertial dynamics | MuJoCo M/bias/qacc and WBC reduced model | orientation, dq and acceleration; periodic in q | raw unwrapped is valid; dq is independent and must not be differentiated from a discontinuously wrapped q |
| C contact geometry | named axisymmetric wheel collision vs floor | phase-periodic; Model B cylinder is continuously symmetric | no absolute-magnitude or revolution requirement |
| D state estimation | MuJoCo Adapter; experimental C620 encoder path | Adapter copies unwrapped qpos with sign/offset; C620 reconstructs multi-turn angle and reports direct speed | current ROS state has no wrap/domain/revolution metadata; a wrapped replacement could create downstream residual jumps |
| E WBC model | canonical-to-native map, closure reconstruction, kinematics/dynamics | raw q is mapped then evaluated periodically | physical model does not require near-equilibrium wheel q; current early validator does |
| F workspace/model validator | `inspectWorkspace` | raw `q-qeq`, wheel ±1 rad | discontinuous under raw wrapping and not periodic; this is the only Phase27 live absolute-magnitude rejection |
| G controller task/reference | Phase27 Minimal/WBC; generic standing/PD profiles | production Phase27 leg residual excludes wheel indices; wheel dq and physical maps remain; generic all-joint PD has raw target-q subtraction | Phase27 has no wheel-angle regulation. Generic joint-PD profiles would be unsafe if fed raw wrapped wheel q without periodic/local reference semantics |
| H logging/visualization | Phase35/40 CSV, ROS conversions, bridge JointState | verbatim q/dq | unwrapped display is safe; accumulated float range and visualization policy need metadata if hardware path is promoted |
| I passive closure/reconstruction | WBC four passive joints and equality closure | wheel q enters periodic forward kinematics; passive solution is geometric | no absolute revolution-count dependency found |
| J x16/NMPC / xi | `WheelAwareNmpcModel` | consumes xi/dxi and base chart, not wheel absolute joint q | wheel joint angle must not be overloaded as xi; wheel spin rate remains a separate material state issue |

## Exact source findings

- `Adapter::extractState` maps MuJoCo qpos and qvel independently without wrap/recenter
  (`ros_ws/src/wheel_leg_mujoco/src/adapter.cpp:98-163`).
- `RobotState` has q and dq only; no revolution count/domain tag
  (`ros_ws/src/wheel_leg_core/include/wheel_leg_core/types.hpp:25-35`). ROS conversions copy both
  arrays verbatim.
- `inspectWorkspace` is the raw subtraction at
  `ros_ws/src/wheel_leg_core/src/nominal_wbc_model.cpp:267-292`; `evaluate` maps the same raw q only
  after this check. The actual kinematic use is `AngleAxis` at lines 104-134.
- Phase27/WBC leg tasks use canonical indices `{0,1,3,4}`; wheel absolute q is not a tracking
  residual. Wheel rate is consumed independently. The x16 NMPC validates xi at state 12/13 and
  does not consume wheel q (`wheel_aware_nmpc_model.cpp:203-240`).
- Generic standing/joint-PD paths do contain ordinary reference-minus-position operations over all
  joints. They are not the Phase27 H0 authority, but they prohibit adopting R1 globally without an
  explicit periodic/local residual contract.

No current model matrix, Jacobian, contact or closure consumer was found that depends on absolute
revolution count. The sole current need for accumulated rotations is measurement/logging/possible
future odometry, for which a separate count is representable but not yet in the formal schema.
