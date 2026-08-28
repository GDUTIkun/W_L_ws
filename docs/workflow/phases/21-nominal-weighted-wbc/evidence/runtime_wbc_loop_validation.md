# P21-T09 runtime WBC loop validation

Date: 2026-08-28. Scope: independent full-3D WBC runner `weighted_wbc_loop`
(`wheel_leg_mujoco`), its Core↔Adapter-only control path, 5-step ZOH, dual
clock, fault/reset entries, replay determinism and non-overwrite. Authoritative
plant: `simulation/mujoco/model/phase18_floating_contact.xml` (nq=17, nv=16,
nu=6, physics 0.002 s, control 0.010 s).

## Root cause found during acceptance (and fix)

Symptom: first control tick solved the weighted QP exactly; from tick 2 the
Core latched to six-zero torque. Diagnosis with the real plant:

- Tick-0 Core torque matched the frozen offline equilibrium QP solution
  (`data/experiments/2026-08-28-phase21-tasks-42d-runtime-v2/summary.json`,
  tau within 1e-5 N·m), so the Core/QP chain was correct.
- Plant CSV showed base height rising 0.35→0.385 m within one control tick and
  bilateral contact lost at the second tick; `contact_state` degraded
  `kContact→kNoContact` and the Core fail-closed latch (correct semantics).
- Isolating with the Phase 20 ground-truth support torque
  (`phase20_formal.json`, support_torque_nm) reproduced the same launch,
  proving the fault was in initial-state setup, not in the WBC.
- Cause: `setInitialState` in the rewritten runner began with `mj_resetData`,
  which restores `eq_active` from the model defaults. The scene's
  `base_weld` equality (from `wheel_leg.xml`) is active by default; the weld
  anchored the base at the XML `qpos0` height (z=0.6) and accelerated the
  floating base upward. `Adapter::reset` had already disabled the weld for
  `floating_base=true` (matching `phase20_equilibrium.json`
  `"disabled_equality": "base_weld"`), and Phase 20's
  `standing_3d_loop.cpp` `setInitialState` never resets a second time.
- Fix: removed the redundant `mj_resetData` from the runner's
  `setInitialState` (reset ordering is now `adapter.reset` →
  `controller.reset` → `setInitialState`, identical to Phase 20). No Core or
  Adapter change was needed.

## Commands and observed results

Run from `ros_ws/` with ROS jazzy sourced:

```text
colcon build --symlink-install --packages-up-to wheel_leg_mujoco
colcon test --packages-select wheel_leg_core wheel_leg_ros wheel_leg_mujoco
colcon test-result --verbose
```

Observed: build PASS (4 packages); `24 tests, 0 errors, 0 failures,
0 skipped`. `git diff --check` PASS.

Runner (from repo root, `ros_ws/install` sourced):

```text
ros2 run wheel_leg_mujoco weighted_wbc_loop \
  --model simulation/mujoco/model/phase18_floating_contact.xml \
  --control-output <dir>/control.csv --plant-output <dir>/plant.csv \
  --scenario <scenario> [--episodes N --ticks N --fault-tick N \
  --disturbance-start-tick N --disturbance-ticks N --force x,y,z \
  --moment x,y,z --initial-state 8 --leg-perturbation 4]
```

Observed (scratch outputs under `/tmp/t09dbg/`, not formal evidence):

- `hold`, 1 episode × 500 ticks (5 s): 500/500 `kOk`, zero latch ticks,
  bilateral `kContact` on every tick, base height band
  [0.3154, 0.3154] m, max |x| drift 4.9e-4 m. Plant truth: max penetration
  5.15e-4 m, max rolling slip 7.43e-3 m/s, max lateral slip 1.54e-3 m/s,
  max closure residual 1.83e-4 m, min wheel normal load 31.38/31.37 N
  (Phase 20 equilibrium: 30.96/32.16 N). Six-way `ctrl` constant within
  every control tick (5-step ZOH exact, `zoh_diff` = 0).
- Replay determinism: identical hold rerun produced a byte-exact `plant.csv`
  and a `control.csv` differing only in the wall-clock `core_step_ns` column
  (500/500 rows); all state/reference/solution/torque columns exact.
- Faults (30 ticks, fault at tick 10): `contact_loss_left`,
  `contact_loss_right`, `invalid`, `nonmonotonic`, `timing` each held `kOk`
  for ticks 0–9, produced the matching fault status at the fault tick
  (`kInvalidState` for invalid, `kNonMonotonicState` for nonmonotonic,
  `kSafetyLatched` for contact loss / timing), then latched with six-zero
  command torque through the end of the run.
- Non-overwrite: rerunning against existing output paths exited with code 1
  and `Refusing to overwrite output`; no existing file was modified.
- Phase 20-envelope disturbances (1000 ticks = 10 s each): `force_x` ±0.1 N,
  `force_y` +0.05 N, pitch moment ±0.01 N·m (10-tick windows), pitch
  initial rotation 0.001 rad, and 0.001 rad four-joint leg perturbation all
  finished 1000/1000 `kOk`, no latch, bilateral contact maintained, height
  band within 1e-4 m of nominal.
- A deliberately out-of-envelope 8 N lateral force for 100 ticks latched
  fail-closed as designed (envelope breach → six-zero); this is expected
  safety behavior, not a PASS case.
- Reset: 3 episodes × 50 ticks, every tick `kOk` in each episode; the
  second/third episodes reproduce the first episode's behavior from reset.

## Input hashes

```text
f27e274ec806d9266e9472850ad2b78f8844c245fcce206357cd2a89ff28620b  ros_ws/src/wheel_leg_mujoco/src/weighted_wbc_loop.cpp
e5b3e73fc1a37639deba4113b2a2938b89947fcce5ce9bdf9f287c982ce84869  ros_ws/src/wheel_leg_mujoco/CMakeLists.txt
8824a7e7cda9a699f27062a0a67e80f621b23f0ef526e0d7beaba184ed126a6d  ros_ws/src/wheel_leg_mujoco/src/adapter.cpp
```

## Scope boundary

This closes the P21-T09 runner deliverable and supplies the Adapter, 5-step
ZOH, dual-clock and runner evidence that `runtime_core_integration.md` listed
as pending for DG21-06. It is not the frozen formal matrix: the 19
normal/perturbation cases, 6 fault cases, per-tick solver/task gates, summary/
manifest/hash and fresh replay/reuse audits belong to P21-T10/T11 and must run
in new `evidence/automated/<run-id>/` directories with frozen inputs.
