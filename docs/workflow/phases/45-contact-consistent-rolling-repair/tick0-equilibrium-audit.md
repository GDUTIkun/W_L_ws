# Tick0 equilibrium audit

Authoritative evidence：`contact-consistent-formal-v3/tick0-equilibrium.csv`，fresh replay-v2 error=0。

- baseline双运行均于tick111首次right contact loss，semantic error=0；
- controller/independent actual-contact row的slip、map、bias max error均为0；
- material tangent acceleration为left `-5.5724e-4`、right `-1.3451e-3 m/s2`，通过0.01门；
- actual ddxi为left `-0.0103356`、right `-0.0533965 m/s2`；right超过冻结0.05门约6.79%；
- native qdd `[-0.09495,-3.08685] rad/s2`只作rolling-response报告，不单独判FAIL；normal load
  `[30.9734,31.5101] N`且双侧task active。

因此 `DG45-EQ=FAIL`。按PLAN不进入AUTH、SHORT、10 s或REAUDIT。
