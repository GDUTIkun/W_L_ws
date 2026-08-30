# Phase 36 REVIEW

结论：**PASS — P36-D_collision_mesh_contact_discretization_artifact**  
日期：2026-08-30

## Gate review

| Gate | Result | Evidence |
| --- | --- | --- |
| DG36-00 semantics | PASS | canonical/native sign、unbounded hinge、rotating collision mesh confirmed |
| DG36-01 static geometry | PASS | 75+75 samples complete; origin invariant, contact phase-sensitive |
| DG36-02 model periodicity | PASS | 15 pairs; core maximum error `3.47e-18` |
| DG36-03 dynamic equivalence audit | PASS | same-torque oracle complete; contact response equivalence FAIL and isolated to contact |
| DG36-04 boundary specificity | PASS | no case meets frozen `5×` transition rule |
| DG36-05 interpretation | PASS | unique P36-D; no repair or gate change |

Formal-v1 is retained as a non-authoritative audit-runner classification defect: it combined core
periodicity with raw contact topology into one scalar and consequently emitted P36-E. Formal-v2
separates those two pre-existing questions without changing the frozen corpus/thresholds. Fresh
replay-v1 is byte-for-byte equal at the parsed summary level.

## Five required answers

1. **Does absolute spin/mesh phase change the plant?** Yes. It materially changes raw contact
   geometry and instantaneous contact-on acceleration, while wheel origin/xi/zeta remain fixed.
2. **Primary source?** The enabled, non-axisymmetric rotating collision mesh and MuJoCo's discrete
   contact-manifold selection. Contact-off phase effect is only `8.75e-5` of contact-on effect.
3. **Is `±1 rad` a special validity transition?** No. No sign/mode reaches the frozen specificity
   ratio; core quantities show no discontinuity or degradation there.
4. **Are `q` and `q+2π` physical-equivalent responses consistent?** Core rigid-body quantities are
   consistent to `3.47e-18`; raw contact topology/instantaneous response is not robustly consistent
   at every pair because discrete contact selection changes.
5. **How should the Phase35 bound be interpreted?** It remains a historical validation envelope,
   not an evidenced necessary one-radian validity contract. The state variable is relevant because
   current collision contact is phase-dependent, but the observed limitation is P36-D artifact and
   does not justify `±1 rad` as its boundary.

## Scope and next direction

Production files, physical parameters, controller and live gate were unchanged. The only allowed
Phase37 recommendation is a dedicated collision-mesh/contact correction study that freezes a
physically justified wheel representation and periodic-contact acceptance oracle before any
workspace-contract change. Selecting that representation is a new technical decision and is not
silently made here.
