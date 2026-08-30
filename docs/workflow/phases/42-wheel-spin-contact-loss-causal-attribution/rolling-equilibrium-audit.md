# Phase 42 Rolling-Equilibrium Audit

## Oracle closure

The frozen MuJoCo convention

```text
M qacc + qfrc_bias
  = qfrc_actuator + qfrc_passive + qfrc_applied + qfrc_constraint
```

closes over every restored snapshot with maximum whole-vector residual `1.279e-13`. Captured versus
restored `qacc` differs by at most `6.899e-12`; the two-width physical-ddxi oracle differs by
`1.721e-9 m/s²`. Per-contact `mj_applyFT` versus point Jacobian/wrench reconstruction is exactly zero
at recorded precision. `qfrc_other_constraint` explicitly retains equality/limit/non-wheel terms;
no `contact.dim` slicing is used.

## Wheel rows

At tick0 post-command, the left wheel row has inertia/contact/actuator contributions
`-8.13659e-5 / -8.11869e-5 / -1.78984e-7`; the right row has
`-5.33742e-4 / -5.33552e-4 / -1.90400e-7`. Other-constraint contributions are below `1.4e-17`.
The corresponding physical wheel-origin accelerations are `-0.0987614/-0.1416514 m/s²`.

Thus the fixed request realized by the production Minimal WBC is not an instantaneous rolling
equilibrium at tick0. This initial acceleration is transient and changes sign later, so it is a
material contribution but is not by itself a complete explanation of the positive late drift.

By tick110, contact coupling remains dominant in both wheel rows: left actuator/contact are
`-0.0266984/+0.0942649`, right are `-0.1235165/+0.1750758`, and physical ddxi is
`+6.00913/+1.71110 m/s²`. Requested/realized WBC interaction wrench and actual MuJoCo contact wrench
remain separately named and are not treated as interchangeable.

