# Phase 20: nominal 完整 3D 简单站立 — REVIEW

Verdict: `PASS`

## Scope Review

- full-3D plant/equilibrium：PASS。authoritative Phase18 scene保持`nq=17/nv=16/nu=6/neq=3`、完整freejoint/闭链/wheel contact，floating reset关闭base weld；本Phase重新求得upright zero-wheel-torque equilibrium。
- state/input/controller：PASS。world FLU quaternion shortest-arc Log、`x8`、common/roll/yaw三路canonical input、单位roll-leg direction、10 ms local model与静态`u=-Kx`均有独立oracle/pre-freeze支持。
- Core/runtime：PASS。新增显式opt-in `kSimpleStanding3d`；旧zero、Joint PD/gravity和Phase19 standing模式未改名。独立C++ runner直接执行Core↔Adapter、2 ms physics/10 ms control/5-step ZOH、reset/fault和逐tick plant/control日志。
- formal/reuse：PASS。formal-v3的19个10 s normal/perturbation cases、6个双episode fault cases、fresh exact replay、non-overwrite、历史回归与fresh-namespace reuse dry-run全部通过。
- scope boundary：PASS。没有真机、WBC/QP/NMPC、absolute-Y task、turning、隐藏constraint、辅助lateral force或公共message/schema修改。

## Evidence Review

最终authority为[`formal-v3`](evidence/automated/2026-08-26-formal-v3/summary.json)，manifest见[`manifest.json`](evidence/automated/2026-08-26-formal-v3/manifest.json)。formal-v1的non-admissible raw-roll reset拒绝与formal-v2缺失plant列均非覆盖保留；二者不作为最终PASS authority。

| Gate | Result | Evidence |
| --- | --- | --- |
| Equilibrium/replay | PASS | max qacc `2.45e-11`；generalized residual `1.51e-10`；closure `1.63e-4 m`；wheel load `30.96/32.16 N`；one-step qvel drift `4.90e-14` |
| State/input authority | PASS | virtual input rank `3`；condition `2.014`；roll cross ratio `1.47e-15`；orientation Log axis error `0` |
| Local/pre-freeze | PASS | controllability rank `8`；training/validation RMS `0.0437/0.0396`；closed-loop spectral radius `0.9910`；全部tuning/holdout通过 |
| Normal/perturbation | PASS | 19/19；worst `|x|=0.00176 m`、`|y-y0|=0.00153 m`、height error`0.000283 m` |
| Orientation/recovery | PASS | worst pitch/roll/yaw `0.00531/0.00476/0.00448 rad`；final linear/angular speed `0.00153 m/s`/`0.0811 rad/s` |
| Contact/plant | PASS | bilateral fraction `1.0`；minimum wheel load `30.17 N`；penetration `0.000525 m`；rolling/lateral slip `0.00954/0.00157 m/s`；closure `0.000185 m` |
| Safety/reset | PASS | left/right contact、invalid quaternion、nonmonotonic、timing、saturation：6/6均从注入tick六路zero并锁存，双episode reset exact |
| Runtime invariants | PASS | ZOH、Adapter sign、virtual mapping errors均精确为`0` |
| Determinism | PASS | primary/replay共25个CSV加summary，26个文件逐一SHA-256 exact；summary hash均为`fc3322f7f684240857003f4de9fee396764fdedd543d723de9293efdfd7aecc3` |
| Compatibility | PASS | colcon 19 tests无失败；coordinate contract、Phase18 plant、Phase19 planar formal回归全部PASS |
| Reuse/non-overwrite | PASS | equilibrium→contract→prefreeze fresh namespace均PASS；非空formal-v3目录返回2且未启动仿真 |

## Findings

Blocking findings: None.

Non-blocking limits:

- authority只覆盖current nominal完整3D MuJoCo与冻结小扰动矩阵；不证明真机、identified profile、参数鲁棒性、region of attraction或跌倒恢复。
- world Y和height是sensed/logged/safety outcome，不是独立Cartesian task；本Phase不声明absolute lateral position regulation。
- heading保持reset首帧yaw，不是absolute compass heading，也不覆盖yaw-rate tracking或turning。
- formal-v1直接roll reset在首个control tick使单轮离地，因此被runner在仿真前拒绝；roll正负方向改由冻结world-X moment覆盖，没有用raw coordinate reset伪造admissible contact state。
- formal-v2虽通过其声明门槛，但缺逐case normal-load/slip/penetration/closure列；REVIEW发现后以formal-v3显式补测并supersede，未把Phase18回归冒充受控站立证据。
- CBM generation仍为`2026-08-26T07:29:33Z`；最终coverage把改动Core/CMake标为metadata changed、new runner标为not tracked。审查已直接通读这些source并以真实build/test/formal为准，不从旧图推断实现完成。

## Conclusion

Phase 20 已完成current nominal完整3D simulation-only简单站立：common wheel稳定X/pitch、差分腿力矩稳定roll、differential wheel保持heading，同时Y/Z、双轮载荷、slip、penetration、闭链与六路torque在冻结范围内有界。可以结束本Phase并进入独立的nominal Weighted WBC Phase；本Phase数值不得直接用于新CAD、identified profile或真机。
