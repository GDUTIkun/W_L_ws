# Phase 36 RECORD

状态：`complete`  
日期：2026-08-30  
最终分类：`P36-D_collision_mesh_contact_discretization_artifact`

Phase36 confirmed that absolute wheel phase is dynamically relevant in the current nominal plant
through its enabled rotating collision mesh, but rejected `±1 rad` as a necessary model-validity
boundary. Fixed wheel origins and core kinematic/dynamic quantities are `2π` periodic to
`3.47e-18`; raw contact centroid varies by up to `54.63 mm`, contact topology can differ even for
numerically equivalent `q/q+2π` transforms, and contact-on ddxi varies by `1.53 m/s²`. Disabling
contact reduces the maximum phase effect to `1.34e-4 m/s²`.

No special repeatable transition exists at one radian. The live gate was neither changed nor
bypassed; Phase35's bound remains a historical validation envelope pending a separate
collision/contact correction Phase.

Authority:

- method hash: `e2c2b28e6120a0c3a9a2e7d1007cacc9d1ca73e4eb9612721302ca50763fafc8`;
- formal: `evidence/automated/wheel-phase-validity-formal-v2`;
- fresh replay: `evidence/automated/wheel-phase-validity-replay-v1`;
- environment: MuJoCo 3.7.0, NumPy 2.2.6, SciPy 1.15.3, repository `./.venv/bin/python`.

Formal-v1 is append-only diagnostic history and is not final authority.
