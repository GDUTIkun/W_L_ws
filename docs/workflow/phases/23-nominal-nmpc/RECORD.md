# Phase 23 Record

Status: `complete`

Review: `PASS` with 0 blocking findings.

## Delivered

- frozen 12D locked-composite base model and 12D external base-FLU wrench input
- acados SQP-RTI + partial-condensing HPIPM OCP (`Ts=20 ms`, `N=20`) generated
  from the frozen RK4 model
- append-only v2 artifact with relative state bounds at stages 1..N and exact
  measured state at stage 0
- private C++ capsule wrapper with project-owned objective, full-horizon defect,
  input/state bound and projected-stationarity audits
- opt-in NMPC+ProxQP Weighted-WBC Core mode with 2:1 update, two-tick wrench ZOH,
  age/status diagnostics, deterministic reset and fail-zero/latch
- versioned method, generator/OCP/reference/formal profiles, per-tick runner
  logs, 23+10 evaluator and complete provenance manifests

## Key results

- generated model next/A/B parity: `4.86e-17 / 4.13e-11 / 3.11e-11`
- component maximum defect/projected stationarity: `3.38e-6 / 0.0428125`
- authoritative normal combined NMPC+WBC maximum: `3.641244 ms < 10 ms`
- authority defect/projected stationarity/input bound/state bound:
  `2.25681e-6 / 0.0250671 / 0 / 0`
- authority and fresh replay: 23/23 normal/reference + 10/10 fault PASS
- replay: 33 plant CSVs byte-exact; control differs only in four declared
  wall-clock fields
- non-overwrite, 99-entry hash audits, coordinate and Phase14/15/18/20 fresh
  regressions: PASS

## Decisions

- no optimizer-only delta-wrench memory is used; the pre-freeze ablation corpus
  did not justify augmentation
- NMPC remains an internal 12D wrench producer; WBC mathematics, ProxQP backend,
  torque extraction, canonical RobotState/TorqueCommand and public ROS schema
  remain unchanged
- ordinary production builds compile the checked-in artifact and require no
  Python, CasADi, `acados_template`, renderer or network access
- solver create/loader failure is a startup environment failure; any runtime
  solve/audit/age/deadline failure produces strict zero torque and latches

## Evidence

- [T06 component](evidence/automated/2026-08-29-phase23-t06-v1/summary.json)
- [Authoritative formal v2](evidence/automated/2026-08-29-phase23-acados-formal-v2/README.md)
- [Fresh replay](evidence/automated/2026-08-29-phase23-acados-formal-v2-replay/README.md)
- [T05 pre-freeze decision](evidence/t05-prefreeze.md)
- [Validation method](../../../experiments/phase23_nominal_acados_nmpc.md)

## Scope limit and next work

This record is current-nominal MuJoCo simulation-host evidence only. The next
ROADMAP entry, Phase 05 actuator/real identification, remains blocked by the
user's real-hardware freeze; the later roll/yaw/turning item has no independent
Phase yet and must not inherit straight-reference thresholds without a new
design and validation gate.
