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

## Phase48-B / P48-T03 — Fixed-H0 Hard Realizability

Status: `PASS / CLOSED`

The authoritative H0 primitive request was reproduced first. It remains hard-infeasible and its
minimum unavoidable normalized L∞ deviation is `0.07832043067340007`, exactly matching the frozen
Phase48-A value within the pre-run `1e-9` tolerance. No semantic, R1, W5 or W1–W6 regression was
observed.

### Request construction

For each wheel, the six canonical columns were projected by the production corrected-R1 projector,
rank-revealed and normalized with the frozen wrench scales. Both projectors have rank 5; the greedy
independent canonical columns are `Fx,Fy,Fz,Tx,Tz`. The requested `Ty` probe is retained as a projected,
dependent physical direction and does not increase the claimed basis rank. The representative matrix
contains common `Fx/Fz/Ty` and differential `Fy/Tx/Tz`, each at normalized magnitude `0.05` on positive
and negative branches. Every primary artificial request has R1-image residual below `1e-9`.

An exact-feasible physical anchor was constructed without soft objectives by maximizing the minimum
normalized hard-row margin subject to the complete hard set and corrected-R1 image. Its minimum row
margin is `0.2985116596376416`. The twelve directional requests are this same fixed-H0 anchor plus the
frozen projected increments; no state, contact, operator or solver setting changes between cases.

### Hard-only formulation and results

The screen reuses the frozen 42D variables and all `117` production rows from the Phase48-A primitive
QP dump. Exact feasibility adds the authoritative physical interaction equality. Infeasible cases use
normalized L∞ minimization followed by a normalized-L2 deterministic tie-break. Production soft costs
are not evaluated or used.

- primary artificial cases: `13`;
- exact hard-feasible: `1` (`P48B-H0-R1-INTERIOR`);
- hard-infeasible: `12` (all frozen ± directional probes);
- untrusted: `0`;
- worst directional minimum normalized L∞: `0.04772509486927712`,
  `P48B-H0-DIFFERENTIAL-TZ-POSITIVE`;
- overall worst including the mandatory nominal regression anchor: `0.07832043067340007`, nominal H0.

The machine-readable rows preserve every 12D request, closest wrench, physical and normalized
deviation, hard/primitive/R1 residual, family margins, active/near-active signature and solver status.
Active rows and dominant deviation channels are observations only; no hard-limit root cause is claimed.

Formal evidence:

- `evidence/automated/phase48-b-hard-screen-formal-v1/`
- `evidence/automated/phase48-b-hard-screen-replay-v1/`

Fresh-process replay has exact request hash, case count and classification, numeric parity within
`1e-9`, and deterministic closest-wrench parity. P48-T04, P48-T05 and P48-T06+ were not entered.

Stop here. P48-T04 requires a separate prompt; preserve the exact-feasible anchor for later P48-T05.

## Phase48-C / P48-T04 — Hard-Infeasibility Attribution

Status: `PASS / CLOSED`; G2 remains `PARTIAL` because P48-T05 eligibility was not checked.

### Selected cases and inventory

The fixed selected set is nominal `P48B-H0-NOMINAL`, worst directional
`P48B-H0-DIFFERENTIAL-TZ-POSITIVE`, and mechanically selected common contrast
`P48B-H0-COMMON-FX-NEGATIVE`. `P48B-H0-R1-INTERIOR` remains the unchanged exact-feasible control.

The authoritative dump contains two active equality families:

- production dynamics: rows `0–11`, 12 rows, rank 12;
- primitive contact response: rows `105–114`, 10 rows, rank 10.

Rows `115–116` are inactive primitive capacity placeholders and are not counted. The full equality
matrix therefore has 22 rows, rank 22 and nullity 20. The implementation also contains torque rows
`12–17`, combined contact cone/unilateral rows `18–91`, and acceleration rows `92–103`; row 104 is an
inactive placeholder. These inequality families were inventoried but not relaxed because no selected
case passed the equality-only-feasible entry gate.

### Equality-reachable wrench subspace

With `K_W=C_W Null(A_eq)`, the equality-reachable wrench rank is 6 versus the frozen 10D corrected-R1
physical request space. The complete singular spectrum is:

```text
18.076142992532123
11.172537127290722
4.070067492473444
0.8905872744036493
0.19692254357935407
0.0224064798696153
8.024266055493385e-16
1.5471417225526134e-16
0, 0, 0, 0
```

Projector and independent direct equality-only solves agree for every selected case:

| Case | normalized L∞ unreachable | equality-only feasible | Primary classification |
| --- | ---: | --- | --- |
| nominal H0 | `0.12552971006032715` | no | `HARD-PRIMITIVE-LAW-STRUCTURAL` |
| differential Tz positive | `0.05005313685764943` | no | `HARD-PRIMITIVE-LAW-STRUCTURAL` |
| common Fx negative | `0.0074550259751212464` | no | `HARD-PRIMITIVE-LAW-STRUCTURAL` |

Nominal minimum normalized L∞ remains `0.07832043067340007`; the T03 catalogue, selected requests,
control feasibility and all T03 classifications are unchanged. Directional full-hard alpha intervals
are not defined because both selected directions fail equality reachability; a linear equality layer
must not be presented as a finite-amplitude boundary.

### Family materiality and the 12/12 result

Leave-one-family-out is decisive but diagnostic only. Removing production dynamics expands reachable
wrench rank from 6 to 9 yet restores none of the selected requests. Removing the 10 active primitive
rows expands rank to 12 and restores direct equality feasibility for all three selected requests, while
leaving the authoritative interaction operator and corrected-R1 projector unchanged. Thus the primitive
hard-law family is a material structural limiter; this does not claim it is a universally unique cause.

All twelve frozen `±0.05` projected directions have nonzero equality-unreachable components (normalized
L∞ range `0.0074550259751212464–0.1341500903050426`). This explains why a positive-margin feasible
anchor can coexist with 12/12 infeasible nearby probes: the anchor lies on a rank-6 affine equality
manifold, while every tested direction leaves that manifold. The result is structural, not an inference
from active torque/cone/acceleration rows and not weighted-task competition.

No BUG-B or numerical inconsistency was found. Restoring the missing authority would require changing
the primitive hard-law/hard-soft architecture, so `ARCHITECTURE DECISION REQUIRED=YES`; no controller
change or repair is authorized here. Formal and hash-identical fresh replay evidence are under:

- `evidence/automated/phase48-c-hard-attribution-formal-v1/`
- `evidence/automated/phase48-c-hard-attribution-replay-v1/`

P48-T05 and P48-T06+ were not entered. Stop here; use a separate prompt for P48-T05 eligibility.
