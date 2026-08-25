# Phase 04 MuJoCo Grounding

Date: 2026-08-25

## Frozen toolchain

- OS: Linux 6.17.0-35-generic x86_64.
- ROS: ROS 2 Jazzy; `colcon-core 0.20.1`.
- Compiler/build: GCC 13.3.0, CMake 3.28.3.
- MuJoCo: exactly 3.7.0 for both `/opt/mujoco-3.7.0` C++ headers/library/tools and the repository-local `.venv` Python package.
- Python reproduction: `uv venv .venv && uv pip install --python .venv/bin/python mujoco==3.7.0`.
- `simulation/mujoco/environment.yml` now pins the same 3.7.0 version. Mixing the former 3.12.0 declaration with the C++ library is not valid Phase 04 evidence.

Both `/opt/mujoco-3.7.0/bin/compile` and Python `MjModel.from_xml_path` loaded `simulation/mujoco/model/scence.xml`. The C++ adapter test also asserts `mj_versionString() == "3.7.0"`.

## Compiled model facts

The machine-readable source is [mujoco_runtime_manifest.json](automated/mujoco_runtime_manifest.json), generated from scene SHA-256 `66cede35ad67dfda852aa15b1e6f4469aa892130a5c9e13761518157aa9f6a1d`.

- `nq=17`, `nv=16`, `nu=6`, 19 sensors / 26 sensor scalars.
- Timestep `0.002 s`; compiled gravity `[0, 0, -9.81]` in canonical FLU.
- Equalities: named `base_weld`, `left_leg_closure`, `right_leg_closure`. The runner changes only `base_weld` at explicit reset to select fixed or floating mode.
- Contact set: `floor` geom 0, `left_wheel_collision` geom 61, `right_wheel_collision` geom 30. Adapter resolution is by name; numeric IDs are evidence, not implementation constants.

| Canonical joint | qpos address | dof address | actuator / ctrl address | gear |
| --- | ---: | ---: | --- | ---: |
| left hip | 12 | 11 | `left_hip_torque` / 0 | 1 |
| left knee | 13 | 12 | `left_knee_torque` / 1 | 1 |
| left wheel | 14 | 13 | `left_wheel_torque` / 2 | 1 |
| right hip | 7 | 6 | `right_hip_torque` / 3 | 1 |
| right knee | 8 | 7 | `right_knee_torque` / 4 | 1 |
| right wheel | 9 | 8 | `right_wheel_torque` / 5 | 1 |

The non-canonical import order demonstrates why the Adapter resolves and caches named addresses rather than assuming qpos order. All actuators have native unit gear; `tau_M=-tau_C` is implemented in Adapter code.

## Grounding findings and decisions

- The included `wheel_leg.xml` previously declared zero gravity. MuJoCo include expansion caused that option to override the outer scene's gravity. Removing the inner option restored the intended compiled `[0,0,-9.81]`; the audit now checks compiled gravity rather than trusting source layout.
- Source simulation time is derived from `mjData.time`. Receipt age is derived from host `steady_clock`; the two domains are never subtracted. This controlled amendment supersedes Phase 03's host-clock-only interpretation without changing message fields.
- A 10,000-step zero-control `testspeed` run completed at 5,760 steps/s (11.52× realtime) with finite solver results. This is a headless capacity check, not realtime certification or dynamics validation.
- Model masses, inertias, friction and collision geometry remain imported/nominal. This grounding proves loadability and interface invariants only.

DG01, DG05 and the model-identification portion of T01 are closed by the exact-version loads, manifest and adapter invariant tests.
