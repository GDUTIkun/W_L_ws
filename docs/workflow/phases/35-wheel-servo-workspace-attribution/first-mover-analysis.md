# Phase 35 first-mover and precedence analysis

## Causal chain

```text
Phase27 Minimal fixed equilibrium-wrench hold
  -> bilateral negative wheel-spin drift (trend tick 9)
  -> right-wheel spin dominates the differential mode
  -> canonical right_wheel delta crosses -1 rad at tick 88
  -> NominalWbcModel::kOutsideWorkspace
```

For the limiting coordinate: reset `q5=0`, trend onset tick 9
`q5=-0.0005354 rad`, virtual marker tick 50 `q5=-0.0485058 rad`, near-boundary tick 87
`q5=-0.9655478 rad` with `+0.0344522 rad` lower margin, and rejecting tick 88
`q5=-1.0743884 rad` with `-0.0743884 rad` margin. Left wheel is `-0.5076926 rad` at rejection.
The wheel-spin common/differential modes change from `0/0` to `-0.7910405/-0.2833479 rad`.

At the last valid sample, common wheel-origin xi changes by `+0.0235625 m` while right-wheel angle
changes by `-0.965548 rad`, giving `Delta q5/Delta xi_common=-40.9782 rad/m`. This is correlation,
not a claim that xi is the gate. Independent raw geometry remains finite at rejection; right
wheel-origin relative xyz is `[0.0267601,-0.2123008,-0.2766665] m`.

Before rejection bilateral contact is continuous; hard violation is at most `7.90e-10`, normalized
slack `6.74e-4`, minimum torque margin `1.99997 Nm`, and stationarity/primal/dual maxima are
`1.63e-8/7.90e-10/1.63e-8`. No contact, hard, slack or torque event precedes the spin trend. The
runner's raw contact/wrench/torque state therefore excludes P35-G/H. DG35-06: **PASS**.
