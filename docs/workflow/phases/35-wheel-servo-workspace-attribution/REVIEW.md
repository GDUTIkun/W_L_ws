# Phase 35: Wheel-Position Servo Workspace Failure Attribution — REVIEW

Status: `review`

## Outcome

Verdict: **PASS**. The Phase34 rejection is uniquely attributed to
`P35-A_pre_target_minimal_wbc_workspace_drift`: under the Phase27 Minimal fixed-wrench hold, bilateral
wheel spin drifts negative from tick 9 and the canonical right-wheel angle crosses the live `-1 rad`
workspace bound at tick 88. No xi task or commanded tracking is required.

## Gate review

| Gate | Result | Evidence |
| --- | --- | --- |
| DG35-00/01 | PASS | single live gate, inspector parity, complete rejecting envelope/raw geometry |
| DG35-02 | PASS/P35-A | H0 `88/88`, H1 `89/89`, fresh replay error `0` |
| DG35-03/04 | causally ineligible | H0 already closes earlier branch; no pulse/H2 formal run |
| DG35-05 | PASS | six Phase34 replays reject at ticks `90–92`, canonical index 5 |
| DG35-06 | PASS | contact/hard/slack/torque events do not precede wheel-spin trend |
| DG35-07 | PASS | no sourced task regulates absolute wheel-spin angle |
| DG35-08 | PASS | activation → spin mode → signed margin → exact gate is reproducible |

## Verification

- Preflight: `./.venv/bin/python`; MuJoCo 3.7.0, NumPy 2.2.6, SciPy 1.15.3; `py_compile` PASS.
- Release build: `colcon build --symlink-install --packages-up-to wheel_leg_mujoco ...`; 4 packages PASS.
- Component/regression tests: 35 tests, 0 errors/failures/skips.
- Authoritative evidence: `evidence/automated/workspace-attribution-formal-v2`; formal-v1 retained and
  superseded because it lacked independent rejecting-tick raw wheel geometry.
- Production invariance: no ControllerCore mode, NMPC/WBC formulation/profile, planner, gain,
  workspace value, solver dimension, hard row or public RobotState/TorqueCommand change.

## Findings

Blocking: none.

Non-blocking: the `±1 rad` wheel bound remains only a sampled runtime-model validity contract. Phase35
does not determine whether that interval is physically necessary for the rotating mesh; the selected
next audit must answer that before P35-I or any repair is considered.

Graphify's bounded incremental code update passed graph-health checks, but semantic ingestion of the
11 Phase35 Markdown reports was unavailable because the configured provider returned HTTP 402. The
current documents/evidence remain authoritative; this is a history-query coverage gap, not a Phase35
technical-evidence gap.
