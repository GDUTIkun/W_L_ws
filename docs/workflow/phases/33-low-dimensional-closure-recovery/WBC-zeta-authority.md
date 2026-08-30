# Gain-Free WBC Zeta Authority

DG33-03: **FAIL (unresolved cross-side isolation)**.

The formal screen used all 24 full-scale `+/-` C1/C2 states from six Phase32 authority samples. For
each state, the Phase33 WBC was perturbed by centered `+/-0.1 m/s^2` left/right desired zeta
acceleration, with half-scale consistency checks. Physical `ddzeta` came from floating-base MuJoCo
qacc and an independent central derivative; WBC model prediction, torque, algebraic realized wrench,
slack and hard residual were recorded separately.

Final authority is `zeta-authority-v3` with byte-identical replay `v4`:

| Gate | Result |
| --- | --- |
| minimum physical self gain `>=0.2` | PASS: `0.6186879390` |
| cross/self `<=0.5` | **FAIL: `0.5125767989`** |
| algebraic realized-wrench change `<=2%` | PASS: `0.340909%` |
| full/half consistency `<=10%` | PASS: `0.020017%` |
| hard violation `<=2e-7` | PASS: `1.49995e-8` |
| solver status | PASS |

The sole worst case is T0 tick 56, C1 negative configuration, left request. It has self gain
`0.621867` and cross response `0.318755`, hence cross/self `0.512577`. This is not P33-A (authority
is material) and not P33-B (wrench preservation passes), so the frozen classification is
`unresolved`.

Runs v1/v2 retained identical numbers but used an overbroad evaluator label “P33-A or P33-B”; that
label is invalidated, not the numeric evidence. v3/v4 correct only the classification and preserve
all thresholds and details.

Per the frozen order, no `kp/kd` set, C1/C2 closure retest or contact revision is admissible in this
Phase after this failure.
