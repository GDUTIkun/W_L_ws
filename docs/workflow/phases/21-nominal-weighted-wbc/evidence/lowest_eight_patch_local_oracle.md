# Phase 21 Lowest-Eight Patch Local Oracle

Date: 2026-08-27  
Verdict: **REWORK — force representability passes, but the frozen selector is not differentially continuous**

## Frozen question

The candidate was not retuned: each wheel selects exactly eight compiled mesh vertices ordered by `(world Z, compiled vertex index)`, with one independent world-frame 3D force per vertex. The first gate asks whether that deterministic force-support selector is also a valid differentiable input to point Jacobians and contact bias. It does not infer eight rigid no-slip constraints from the eight force points.

Fresh evidence:

- state/geometry capture: `data/experiments/2026-08-27-phase21-patch-local-capture-v2/`
- selector local oracle: `data/experiments/2026-08-27-phase21-patch-local-oracle-v2/`
- independent wrench replay: `data/experiments/2026-08-27-phase21-contact-representation-audit-v4/`
- config: `simulation/mujoco/config/phase21_patch_local_oracle.json`
- tools: `tools/experiments/capture_weighted_wbc_contact_representation.py`, `tools/experiments/validate_weighted_wbc_patch_local_oracle.py`, and the independent `tools/experiments/audit_weighted_wbc_contact_representation.py`

The authoritative capture refreshes MuJoCo derived pose/contact fields at the start of each control tick, then records matching `qpos/qvel`, wheel-geom pose, and validation-only contact truth. Pose reconstructed independently from saved `qpos` matches the capture exactly. The baseline first solver failure remains tick 272.

## Results

Equilibrium and all three frozen Phase-15 workspace samples produce finite, exactly repeatable eight-index selections. Across rolling ticks 1–271, however, the selected ordered tuple changes 59 times on the left and 60 times on the right. Nine left and ten right transitions replace members of the selected set rather than merely permuting the same eight vertices.

For every switch the oracle evaluates both the previous and new ordered vertex identities at the same new-tick wheel pose. This removes ordinary body motion from the discontinuity measurement. At true set switches:

| Quantity | Maximum |
| --- | ---: |
| same-pose slot position jump | `0.0449267789 m` |
| same-pose canonical reduced point-Jacobian jump | `0.0402586210` |
| 10 ms finite-tick contact-bias jump proxy | `14.5259426 m/s^2` |

The position and Jacobian continuity gates use `1e-9` only as a numerical zero test; observed failures exceed it by roughly seven to eight orders of magnitude. This is not a relaxed differential threshold or a parameter sweep. Full old/new indices, rolling joint angle, same-pose points, 3-by-12 reduced Jacobians, and the finite-tick bias proxy are saved in `switches.json`.

At a member-replacement boundary, the ordered selector maps a slot from one spatially distinct material vertex to another. Consequently the selected point and its reduced Jacobian are discontinuous and a classical finite `Jdot_nu` for that selector is undefined at the switch. Deterministic tie-breaking does not repair differential continuity.

The independent v4 wrench replay still finds the lowest-eight force distribution feasible in all 540 valid wheel-side samples at the frozen `1e-7` wrench residual. Thus the result is deliberately split:

- force-patch resultant representability: **PASS**;
- selector geometry/Jacobian/contact-bias semantics: **FAIL**.

## Gate decision

The frozen lowest-eight candidate is rejected without changing the selector, vertex count, support band, friction, tasks, solver, or thresholds. Because gate 1 failed, the patch Pfaffian/static dynamics oracle, 78D hard QP, independent 78D solver cross-check, dense-ADMM benchmarks, and 36D-to-78D deadline comparison were not run. This is the required early stop, not missing positive evidence.

DG21-01/02 remain `REWORK`. DG21-03/04 are reopened for any future contact representation; their 36-variable results remain historical only. P21-T03 remains `doing`, while P21-T04/T05/T06 and production Core integration are blocked on a new, separately frozen continuous contact representation. No `RECORD.md` is permitted.

## Follow-up

The authorized continuous-representation follow-up is recorded in [continuous_contact_representation.md](continuous_contact_representation.md). It does not alter this failure verdict: the discrete lowest-eight candidate remains rejected. The new analytic six-point surface patch and separate three-row-per-wheel soft Pfaffian pass their local oracles, while QP/Core work remains blocked on the projected-wrench/static-dynamics contract.
