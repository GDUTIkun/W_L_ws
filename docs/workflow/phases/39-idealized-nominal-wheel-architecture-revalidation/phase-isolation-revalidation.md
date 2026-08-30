# Phase-isolation revalidation

DG39-01：`PASS`。

Model B 上 frozen Phase 37 corpus 的 contact centroid、normal、depth 最大变化分别为
`2.776e-17 m`、`0`、`6.939e-18 m`，contact count 不变。core `2π` periodic error 为
`6.776e-20`，dynamic response periodic error 为 `2.518e-13`。

contact-on physical `ddxi` phase effect 为 `2.861e-8 m/s²`，contact-off 为
`9.538e-8 m/s²`；on/off ratio `0.2999`，低于冻结的
`max(0.001 m/s², 10 × off)` isolation gate。相对 Phase 36 mesh plant 的 effect ratio 为
`1.870e-8`。这关闭 material absolute wheel-phase sensitivity，同时保留 wheel-rate/rolling-slip
作为独立动力学变量。

formal-v2 与 replay-v2 的 phase-isolation summary 完全相等。
