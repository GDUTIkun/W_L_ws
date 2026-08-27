# Phase 21 Rolling-Contact Representation Decision Audit

Date: 2026-08-27  
Verdict: **representability PASS only; the candidate is rejected by the superseding differential-continuity oracle**

## Question

The failure-attribution audit showed that the fixed-COP and world-offset single-force models depart from the mesh-contact plant before the tick-272 numerical failure. This audit asks the narrower next question: can contact truth be represented from canonical-state wheel geometry without reading MuJoCo contacts online, and what is the minimum supported structure?

This is a validation-only replay. MuJoCo per-contact force, moment and COP are outputs used by the offline oracle and are never fed to the controller.

## Frozen inputs

- Controller/model/QP: unchanged Phase21 task-prefreeze v5 with model v8 and the frozen dense ADMM settings.
- Window: baseline ticks 1–271, ending before the first tick-272 solver failure.
- Geometry input: compiled current-nominal left/right wheel-mesh vertices and each wheel geom pose reconstructed from the same canonical plant state.
- Candidate support patch: vertices within `1 mm` of the lowest world-Z mesh vertex; `0.1/0.5/1 mm` bands are reported separately as sensitivity, not controller tuning.
- Force cone: independent point forces with world tangential pyramid `|Fx|,|Fy| <= Fz`, `Fz >= 0`, `mu=1`.
- Wrench oracle: SciPy HiGHS minimizes the infinity-norm error between the truth `[F,M_about_wheel_center]` and the candidate point-force resultant.

Fresh evidence:

- capture: `data/experiments/2026-08-27-phase21-contact-representation-capture-v1/`
- authoritative audit: `data/experiments/2026-08-27-phase21-contact-representation-audit-v3/`
- config: `simulation/mujoco/config/phase21_contact_representation_audit_v2.json`
- capture/oracle tools: `tools/experiments/capture_weighted_wbc_contact_representation.py` and `tools/experiments/audit_weighted_wbc_contact_representation.py`

## Results

There are 537 valid wheel-side samples. The 1 mm support envelope contains the normal-load-weighted truth COP in every sample.

| Candidate | Feasible fraction at `1e-7` wrench error | Maximum wrench error |
| --- | ---: | ---: |
| fixed v8 material point | not a force-distribution candidate | moment residual `0.871 N·m` |
| support-patch centroid point | not a force-distribution candidate | moment residual `0.617 N·m` |
| four lowest vertices | 67.78% | `9.78e-4` |
| four support-envelope extremes | 99.44% | `3.41e-4` |
| all vertices in the 1 mm support patch | 100% | `0` |
| deterministic lowest eight mesh vertices | 100% | `0` |

Even an arbitrary single pure force has an irreducible force-parallel moment: median `0.00162 N·m`, p95 `0.01252 N·m`, maximum `0.05170 N·m`. More importantly, choosing a geometry-only centroid does not predict the load-dependent COP: its COP error is median `4.67 mm`, p95 `6.13 mm`, and maximum `20.16 mm`.

The `0.1 mm` support band contains only `8.94%` of truth COPs. Both `0.5 mm` and `1 mm` bands contain `100%`; the 1 mm band is retained because it is the declared audit input and matches the current contact-compliance width order. This does not authorize use of plant contact state in the runtime model.

## Decision

Changing from one selected point to another is rejected. The next **local model-oracle candidate** is a canonical-state, state-dependent wheel support patch:

1. transform the frozen current-nominal compiled wheel vertices by the reconstructed wheel pose;
2. order vertices deterministically by `(world Z, compiled vertex index)`;
3. expose the lowest eight vertices per wheel as candidate force application points;
4. assign an independent `[Fx,Fy,Fz]` force with unilateral/friction bounds to each point;
5. sum `J_i^T f_i` for reduced dynamics and sum each point-force wrench about the base for interaction-wrench fidelity.

The fixed eight-point form is selected over the variable-size 1 mm set because it has the same 100% oracle result while giving a deterministic matrix size. It is selected over four points because four-point residual exceeds the existing hard-residual scale.

This candidate changes contact-force dimension from 6 to 48 and would change the complete QP from 36 to 78 variables. Therefore it is **not yet a production or solver freeze**. Before modifying the existing 36-variable solver or Core, it must pass a new local/static/virtual-work/contact-bias oracle and a pre-freeze 78-variable hard-QP feasibility/deadline decision. The contact acceleration task point is also still open; force-patch success must not be used to silently define its Pfaffian/Jdot semantics.

DG21-01/02 remain `REWORK`, P21-T03 remains `doing`, and P21-T06 remains blocked. DG21-03/04 retain only their historical 36-variable result and cannot be inherited by a 78-variable candidate.

## Superseding local-oracle result

The required next gate has now run; see [lowest_eight_patch_local_oracle.md](lowest_eight_patch_local_oracle.md). The selector remains deterministic and the independent fresh replay still gives 100% force-resultant representability, but rolling set-member replacements cause a same-pose point jump of `44.93 mm`, a canonical reduced Jacobian jump of `0.0403`, and an undefined classical `Jdot_nu` at the switch. Therefore the lowest-eight candidate is rejected and no 78D QP or solver benchmark is authorized.
