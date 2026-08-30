# Wrapped-state discontinuity audit

For each left/right/bilateral mode, raw states at `π±1e-6` and `-π±1e-6` were compared with their
wrapped equivalents. Physical transforms, M/bias/J, closure, contact/load, qacc and ddxi retain
periodic equivalence within the `1e-8` gate. DG40-03 wrapped physical equivalence: **PASS**.

The raw wrapped coordinate changes by `6.283183307179586 rad` across the positive π seam. Exact
software consumer causing the live discontinuity risk:

```text
NominalWbcModel::inspectWorkspace: delta = q - qeq
generic standing/joint PD: error = qref - q
```

The Phase27 physical WBC does not need this jump. Therefore R1 is not safe as an unconditional
RobotState replacement. A consumer must use a periodic error, a continuous local state, or a
separate wrap epoch; dq must remain independently measured rather than differentiated through the
seam.
