# P21-T07 Runtime Model and Solver Implementation Handoff

Date: 2026-08-28
Status: **superseded — implementation and repaired runtime acceptance completed by Codex;
see `runtime_workspace_gate_repair.md` and `runtime_cpp_parity.md`**

## Live-code grounding

CBM project `W_L_ws`, generation `2026-08-27T09:12:24Z`, and direct source inspection
confirm that `wheel_leg_core` already contains the audited Eigen-only
`wheel_leg::DenseQpSolver` with fixed capacity `42` variables and `128` constraint rows.
Its public API is `setup/reset/solve`, with explicit cold/warm modes and fail-zero rejected
results. The package currently has no reduced-model, passive-reconstruction, WBC problem or
wrench implementation. `ControllerCore` still exposes only zero, PD/gravity and Phase-20
simple-standing modes; it is outside P21-T07.

The pre-change baseline passes:

```text
cd ros_ws
colcon build --packages-select wheel_leg_core --event-handlers console_direct+
colcon test --packages-select wheel_leg_core --event-handlers console_direct+
colcon test-result --verbose --test-result-base build/wheel_leg_core
```

Result: one package built; `wheel_leg_core_contract` and
`wheel_leg_core_dense_qp_solver` both pass.

## Frozen implementation route

Production code remains C++17/Eigen-only and must not link MuJoCo, parse MJCF or read plant
truth. No online finite difference is allowed. Implementation is split into two reviewable
steps.

### Step A — deterministic offline parameter profile

Claude owns only:

- `tools/experiments/export_weighted_wbc_runtime_profile.py`
- `simulation/mujoco/config/phase21_runtime_model_profile_v1.json`

The exporter resolves the authoritative Phase-21 model/config chain and emits deterministic
compiled tree data: gravity; all 11 bodies with parent transform and compiled inertial;
joint positions/axes/addresses; base-control and closure sites; active/passive/actuator
orders and signs; equilibrium branch; Phase-15 workspace/reconstruction gates; continuous
wheel-contact geometry; and the fixed 37-row contact-centred H-cone. It records hashes for
the scene, included MJCF, config chain, Phase-14 parameter manifest, Phase-15 geometry
manifest and equilibrium. It refuses overwrite, rejects non-finite or dimension mismatch,
cross-checks all inertials against Phase 14, and must reproduce byte-identically in a second
fresh run. This step creates no C++ and makes no new mathematical choice.

### Step B — runtime-independent C++ components

After Codex accepts Step A, Claude owns these new files plus the corresponding additive
entries in `wheel_leg_core/CMakeLists.txt`:

- `include/wheel_leg_core/nominal_wbc_model.hpp`
- `src/nominal_wbc_model.cpp`
- `include/wheel_leg_core/weighted_wbc_problem.hpp`
- `src/weighted_wbc_problem.cpp`
- `include/wheel_leg_core/weighted_wbc_controller.hpp`
- `src/weighted_wbc_controller.cpp`
- `test/test_nominal_wbc_model.cpp`
- `test/test_weighted_wbc_problem.cpp`
- `test/test_weighted_wbc_controller.cpp`

`ControllerCore`, public `RobotState/TorqueCommand`, MuJoCo packages and old controller modes
remain untouched until P21-T08.

The model uses a 16D unconstrained tree tangent: world-axis base-control-site linear/angular
velocity followed by ten native hinge rates. It reconstructs four passive positions on the
frozen Phase-15 branch with analytic closure residual/Jacobian and fail-closed workspace,
singular-value, condition and residual gates. Passive velocity and acceleration bias are
analytic. With the `16x12` tangent reduction `N`, the frozen reduced equations are

```text
M_r = N' M_16 N
h_r = N' (h_16 + M_16 Ndot_nu)
S_r = N' B
```

where canonical active position, velocity and torque are the negative of native MuJoCo
signs. Full-tree kinematics, inertia, velocity products, gravity, material-point contact
Jacobian and `Jdot_nu` are evaluated analytically from the frozen profile; numerical
differentiation is test/oracle-only.

The problem component assembles the already-frozen 42D order and 104 hard rows, then the
seven explicitly accounted soft costs (including the independent slack penalty). The
controller component is a thin wrapper around the existing `DenseQpSolver`; it extracts
canonical torque and diagnostics but does not implement Core latching or mode selection.

## Acceptance order

1. Step-A schema, provenance, inertial cross-check and byte determinism.
2. Reconstruction branch/closure/conditioning and analytic passive `qdot/qddot_bias`
   against fresh Python/MuJoCo golden data.
3. Per-case `M_r/h_r/S_r`, contact Jacobian/bias, wrench transforms and H-cone parity.
4. Exact scaled `H/g/A/l/u` parity on equilibrium plus the frozen 32-case corpus.
5. Existing dense-solver cold/warm corpus parity, hard reject and non-finite/fault tests.
6. From `ros_ws/`, `colcon build`, `colcon test`, and verbose test-result with zero failures.

A passing build is not model evidence. Codex must read the golden/parity outputs before
marking P21-T07 done or opening P21-T08.

## Current execution blocker

Claude Code `2.1.234` is installed, but the user-level settings inject an external
`ANTHROPIC_AUTH_TOKEN` that the client rejects because it contains non-ASCII characters.
Excluding user settings removes that token but leaves no logged-in provider. Both attempts
fail before model invocation and create no owned files. This is an implementation-worker
environment failure, not a Phase-21 model/control evidence failure. Resume Step A after the
Claude authentication is repaired; do not route implementation to a Codex `phase_worker`
while the repository's temporary worker policy remains active.
