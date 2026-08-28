# Phase 22 dependency and solver profile evidence

Date: 2026-08-28  
Scope: P22-T01/P22-T02 and DG22-01/DG22-02 pre-production evidence

## Dependency identity

- source: `/home/t/opt/proxsuite`, clean tag `v0.7.3`, commit `b93d7778ffc3299d84b5cb0851022a29bf24a596`
- installed CMake config: `/usr/local/lib/cmake/proxsuite/proxsuiteConfig.cmake`, SHA-256 `d3aa4999b2c7a63e7617bb4e66327f1fb7f1e28a9148868e6e513755c432f30e`
- installed/source `proxqp/dense/dense.hpp`: identical SHA-256 `1f6bdbec69774173926f401742ab5d9cafca57b1e90617fb4fee47456ec93c99`
- exported target: `proxsuite::proxsuite`; production does not use source/build include paths or the vectorized target
- compiler: GCC 13.3.0; Eigen package 3.4.0-4build0.1
- `cmake --find-package ... -DMODE=EXIST`: PASS
- `ros2 pkg prefix proxsuite`: package not found. This non-runtime CMake package is therefore accepted only through clean CMake/colcon consumption, not ROS resource-index enumeration.
- `pkg-config proxsuite`: unusable because the installed metadata requests `simde.pc`, which is absent. The project intentionally does not use this discovery path.

The clean `--packages-up-to wheel_leg_mujoco` Release build with
`find_package(proxsuite 0.7.3 EXACT CONFIG REQUIRED)` and
`proxsuite::proxsuite` passed. The isolated negative configure probe with
`CMAKE_DISABLE_FIND_PACKAGE_proxsuite=TRUE` failed at configure time with
`A REQUIRED package cannot be disabled` (exit 1), proving there is no silent
fallback. DG22-01 is closed.

## Exact-corpus pre-production profile

Input is the unchanged Phase 21 32-case corpus:
`data/experiments/2026-08-28-phase21-weighted-solver-runtime-v3/problem_corpus.txt`.
The profile executable independently performs exact bound-row classification, ProxQP init/update/solve, KKT stationarity reconstruction, oracle objective and physical-torque comparison. It does not call the production `DenseQpSolver`.

Frozen profile: `simulation/mujoco/config/phase22_proxqp_solver_v1.json`.
Release command:

```bash
cd /home/t/W_L_ws/ros_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select wheel_leg_core \
  --cmake-clean-cache --cmake-args -DBUILD_TESTING=ON -DCMAKE_BUILD_TYPE=Release
./build/wheel_leg_core/profile_proxqp_solver \
  ../data/experiments/2026-08-28-phase21-weighted-solver-runtime-v3/problem_corpus.txt
```

Observed over 1000 cycling problems per path:

| Metric | Result | Gate |
| --- | ---: | ---: |
| cold maximum setup+solve | 0.832765 ms | <= 10 ms |
| dynamic warm maximum update+solve | 0.256366 ms | <= 10 ms |
| cold / warm maximum iterations | 16 / 9 | <= 10000 |
| maximum ProxQP primal residual | 9.7655862e-9 | audit |
| maximum ProxQP dual residual | 2.9134812e-9 | audit |
| independent stationarity infinity norm | 2.9134814e-9 | <= 2e-7 |
| bound/equality violation | 9.7655862e-9 | <= 2e-7 |
| maximum scaled-x oracle difference | 3.0933395e-5 | diagnostic |
| maximum physical torque difference | 2.8314025e-5 N m | <= 5e-4 N m |
| maximum objective gap | 2.4415758e-9 | <= 2e-6 |

Result: PASS. `PrimalDualLDLT`, init preconditioning enabled, update preconditioning disabled, `eps_abs=eps_rel=1e-8`, `max_iter=10000`, and the complete v0.7.3 settings snapshot are frozen in the versioned profile. The unoptimized first probe was intentionally non-authoritative: its numerical gates passed but its timing failed, demonstrating that Release identity is part of the timing claim.

This closes DG22-02 for adapter implementation. Allocation behavior and repeated-same warm determinism remain component acceptance items under DG22-03; they are not inferred from this pre-production profile.
