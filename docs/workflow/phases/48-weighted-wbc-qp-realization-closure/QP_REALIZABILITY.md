# Phase 48-A — Baseline and Semantic Integrity

Status: `PASS / CLOSED`

## Scope and Verdict

P48-T01/P48-T02 only audited the authoritative chain and fresh-reproduced the frozen H0 anchors. No
new wrench family, magnitude scan, constraint attribution, task attribution, tuning, trajectory,
AUTH/REAL/SHORT/10 s or NMPC work was entered.

```text
AUTHORITATIVE CONTROL PATH: UNIQUE
PROFILE IDENTITIES: FROZEN
W_REF / W_WBC / TAU / W_MJ SEMANTICS: PASS
COMMON WRENCH COORDINATE CONTRACT: PASS
TEMPORAL / STATE PROVENANCE: PASS
NO KNOWN SEMANTIC BUG: YES
CLEANUP REGRESSION: NO
ARCHITECTURE DECISION REQUIRED: NO
PHASE48-A: CLOSED
```

## Fresh H0 Results

| Metric | Fresh result | Gate |
| --- | ---: | --- |
| baseline maximum normalized slack | `0.001522220395389018` | PASS |
| primitive-R2 maximum normalized slack | `0.05850370867784012` | reproduced; historical EQ remains FAIL |
| dominant channel | right `Tx` | reproduced, not attributed here |
| dominant baseline normalized slack | `1.5222939650929801e-6` | reproduced |
| dominant primitive-R2 normalized slack | `-0.05850370867784012` | reproduced |
| dominant delta | `-0.05850523097180521` | reproduced |
| W_ref primitive-feasible | `NO` | reproduced; no new hard scan |
| minimum unavoidable normalized L∞ | `0.07832043067340007` | reproduced with original formulation |
| W5 operator residual | `1.0658141036401503e-14` | PASS |
| W5 offset residual | `2.6645352591003757e-15` | PASS |
| slack reconstruction max error | `0.0` both profiles | PASS |

W1–W6 are PASS. The primitive decision-row rank and hard-rank increment are `10/10`. The 42D witness
is PASS with:

- solver: `SOLVED`;
- hard residual: `3.623378202098832e-9`;
- minimum inequality/cone margin: `0.2197161714633595`;
- minimum torque margin: `1.9990801609079853 N m`;
- corrected R1 residual: `1.2258392916278746e-14`;
- primitive-law residual: `2.74192713867194e-7`;
- candidate predicted row-force margin: `3.5496361436896655`.

`COMP=PASS`; Phase46 remains exactly `EQ=FAIL`, `AUTH/REAL/SHORT/10 s=NOT ENTERED`.

## Profiles and Provenance

- baseline: `phase46_point_realizable_rolling_v1` / `R46P-H0` /
  `kPhase46PointRealizableRolling`;
- primitive-R2: `phase46_mujoco_contact_response_v1` / `R46M-H0` /
  `kPhase46MujocoContactResponse`;
- model: `scene_axisymmetric_centered_com_v1.xml`;
- interpreter/dependencies: `/home/t/W_L_ws/.venv/bin/python`, MuJoCo `3.7.0`, NumPy `2.2.6`,
  SciPy `1.15.3`.

`W_ref`, `W_WBC` and `tau` share the same fixed-H0 QP input snapshot. `W_MJ` is the constrained
reaction obtained after applying `ctrl=-tau` at unchanged pre-command qpos/qvel and running the
constraint evaluation without integration; it is not a next-state reaction. See
[REALIZATION_METHOD](REALIZATION_METHOD.md) and machine-readable `provenance.json`.

## Replay and Evidence

Fresh replay is exact after excluding the intentionally nondeterministic `wbc_time_s`:

- baseline CSV semantic maximum error: `0.0`;
- primitive decision JSON: exact equal;
- slack/minimax decision JSON: exact equal.

Primary evidence:

- [semantic baseline](evidence/automated/phase48-a-semantic-baseline-formal-v2/semantic-baseline.json)
- [12D component vectors](evidence/automated/phase48-a-semantic-baseline-formal-v2/wrench-components.csv)
- [temporal provenance](evidence/automated/phase48-a-semantic-baseline-formal-v2/provenance.json)
- [fresh replay summary](evidence/automated/phase48-a-semantic-baseline-formal-v2/fresh-replay-summary.json)
- [primitive formal](evidence/automated/phase48-a-primitive-formal-v1/r2-mujoco-primitive-contact-law-repair.json)
- [original-formulation minimax](evidence/automated/phase48-a-slack-formal-v1/r2-mujoco-primitive-contact-law-repair.json)

The baseline capture was produced through the historical point-realizable equilibrium wrapper. That
wrapper returns `2` because it also evaluates an old pre-corrected-R1 component criterion that is not a
Phase48-A gate; its fresh H0 row, equilibrium metrics and replay are valid and exact. The current
authoritative corrected-R1 regression is supplied by the primitive W1–W6/witness chain and remains
machine-scale. The obsolete extra verdict is not promoted into Phase48 authority.

## Next Action

Stop here. A separate prompt may start P48-T03 / Phase48-B fixed-state hard wrench realizability.
