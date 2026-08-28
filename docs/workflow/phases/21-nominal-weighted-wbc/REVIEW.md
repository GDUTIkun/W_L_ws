# Phase 21: nominal Weighted WBC — REVIEW

Verdict: `PASS`

## Scope Review

- reduced model/QP：PASS。runtime-independent C++路径实现冻结的12-DoF reduced model、42变量/104 hard rows、连续contact-centred wrench、interaction-wrench slack及workspace fail-closed；production Core不链接MuJoCo。
- solver/task：PASS。project-owned Eigen-only dense ADMM按冻结`alpha=1.6`、`rho=0.15`运行；model/problem golden parity、hard/equality/stationarity、物理力矩与task/slack gates均通过。
- Core/runtime：PASS。新增显式opt-in `kWeightedWbc`，保持canonical `RobotState -> TorqueCommand`、10 ms control/2 ms physics/5-step ZOH、Adapter符号、fault latch/reset及旧mode兼容性。
- formal/reuse：PASS。19个10 s normal/perturbation、6个双episode fault、fresh replay、non-overwrite、Phase14/15/18/20历史回归均通过。
- scope boundary：PASS。没有引入NMPC、真机、identified profile、terrain、隐藏外力/约束或公共message schema改动；simulation deadline不解释为目标硬件实时性。

## Evidence Review

最终authority为[`formal-v1`](evidence/automated/2026-08-28-formal-v1/summary.json)，manifest见[`manifest.json`](evidence/automated/2026-08-28-formal-v1/manifest.json)，fresh replay见[`formal-v1-replay`](evidence/automated/2026-08-28-formal-v1-replay/summary.json)。P21-T12重新核对两套manifest：每套67项config/runner/wrapper/scene/source/output hash，合计134项全部匹配当前文件。

| Gate | Result | Evidence |
| --- | --- | --- |
| Model/problem parity | PASS | 32-case model/problem parity；runtime model workspace与dynamic cases通过；production library无MuJoCo依赖 |
| Solver | PASS | cold/dynamic最大总时长`8.273542/8.790942 ms`；hard/equality/stationarity最大`1.128e-7/1.128e-7/4.124e-8`；物理力矩差`3.075e-5 N·m` |
| Workspace/hard layers | PASS | capture tick 1–259 in-workspace；28个dynamic nominal与4个workspace case全部PASS；260/271按合同拒绝；minimum cone/torque margin`0.310102/1.99854 N·m` |
| Weighted tasks/pre-freeze | PASS | local algebra、32-case competition、4个10 s tuning与9个holdout全部PASS；workspace failure/violation均为0 |
| Normal/perturbation | PASS | 19/19；worst X/Y`2.075e-3/2.006e-3 m`、height`1.680e-4 m`、roll/pitch/yaw`6.377e-3/7.181e-3/1.977e-2 rad`、leg`1.480e-2 rad` |
| QP/task runtime | PASS | hard/primal/dual/stationarity`1.070e-7/1.265e-7/6.506e-8/4.205e-8`；slack`3.728e-3`；task residual/cost`5.523e-3/4.290e-5` |
| Contact/plant | PASS | bilateral fraction`1.0`；minimum load`31.27 N`；penetration`5.369e-4 m`；rolling/lateral slip`8.272e-3/1.731e-3 m/s`；closure`1.835e-4 m` |
| Safety/reset | PASS | left/right contact、invalid、nonmonotonic、saturation、timing共6/6按注入语义六路zero并锁存；双episode reset exact |
| Timing/runtime invariants | PASS | primary max Core step`9.90157 ms <= 10 ms`；5-step ZOH与Adapter sign error均为0；零饱和 |
| Determinism/integrity | PASS | 25个plant CSV字节一致；control仅`core_step_ns`墙钟列不同；summary排除墙钟后相等；134项manifest hash复核无漂移 |
| Compatibility | PASS | P21-T12从`ros_ws`复建4 packages；workspace汇总`24 tests, 0 errors, 0 failures`；Phase14/15/18/20 fresh regressions PASS |
| Non-overwrite | PASS | 已存在formal目录返回2，52个文件清单不变 |

## Findings

Blocking findings: None.

Non-blocking limits:

- authority仅覆盖current nominal完整3D MuJoCo、冻结workspace及小扰动case matrix；不证明真机、identified/new CAD profile、参数鲁棒性、region of attraction、单轮支撑、terrain或跌倒恢复。
- primary formal的最坏Core step为`9.90157 ms`，距离10 ms gate仅约`0.098 ms`；fresh replay最坏为约`7.93 ms`。这满足冻结simulation-host gate，但余量薄，不能推断树莓派或真机实时性。
- 42D解含接触内力平坦方向，因此solver等价性以hard/task/objective及物理力矩为准；冻结problem matrix本身仍保持golden parity。
- CBM generation `2026-08-28T02:59:23Z`已找到WBC符号，但Core关键文件显示`metadata_changed`、新runner显示`not_tracked`，`tools/`按规则排除。审查已直接读取live source，并以当前source hash、真实build/test及formal manifest为准。

## Conclusion

Phase 21已完成current nominal simulation-only Weighted WBC：从canonical state进行闭链重构与12-DoF建模，求解42D加权QP并经canonical torque边界闭环，在冻结normal/fault矩阵内同时满足solver、task、contact、plant、reset、replay与兼容性门槛。DG21-06/07/08关闭，可以创建RECORD并结束本Phase；后续NMPC、identified profile或真机验证必须独立立项。
