# Phase 15: MuJoCo 完整闭链运动学与 Jacobian 验证 — REVIEW

Verdict: `PASS`

## Reviewed Scope

- Phase 15 PLAN 的 Goal、Frozen Decisions、DG01–DG06、T01–T10 和 Acceptance Criteria。
- `phase15_nominal.json` 的 geometry/profile、工作域、solver、epsilon 和预冻结阈值。
- `run_mujoco_closed_chain_kinematics.py` 的被动求解、独立 FK/full Jacobian、约束降维、finite difference、速度/虚功、左右契约、确定性和非覆盖行为。
- 正式 `geometry_manifest.json`、`run_manifest.json`、`phase15_validation.json`、210 行 `workspace.csv`。
- Phase 02/04 坐标回归和 Phase 14 全套内部动力学回归。

## Goal and Gate Review

| Area | Evidence | Result |
| --- | --- | --- |
| Geometry/contact profile | profile↔compiled position `5.5511e-17 m`；mesh/nominal radius `1.2075e-4 m`；方向误差 `5.5511e-18` | PASS |
| Assembly branch | 左/右 7 个 knee continuation；最大 closure `2.8311e-15 m`；reverse `1.1202e-13 rad` | PASS |
| Workspace/singularity | 210/210 样本保留；最小 passive singular value `7.3709e-3`；最大 condition `30.1993` | PASS |
| Independent FK/full J | position `2.7756e-16 m`；rotation `6.6613e-16`；full J `6.0490e-16` | PASS |
| Reduced J/finite difference | `Jc·S=1.3341e-16`；analytic↔MuJoCo `6.0490e-16`；formal FD linear/angular `1.4622e-11 / 1.7825e-11` | PASS |
| Velocity/virtual work | velocity `9.9274e-17`；virtual work `2.2204e-16 N·m`；power `2.7756e-17 W` | PASS |
| Left/right | mirrored position `2.4980e-16 m`；shared-frame rotation error `0` | PASS |
| Reuse/non-overwrite | smoke/formal checks 与 CSV 完全一致；重复正式命令 exit 1；manifest hash 匹配 | PASS |
| Regression | coordinate contract PASS；Phase 14 九项 PASS；历史 Phase 14 evidence 未修改 | PASS |

## Findings

### Blocking

None.

### Non-blocking / Accepted Limits

- 名义 contact point 是从 current nominal collision mesh 归一化得到的可微圆周点，不是 MuJoCo 瞬时 contact manifold，也不是实测轮胎半径。
- 本 Phase 验证固定基座几何和闭链速度空间；floating-base 接触动力学、摩擦、滑移和地面反力不在本次 PASS 范围。
- 工作域结论只覆盖 config 冻结的 `hip/knee/wheel` 网格；模型 revision 改变后必须使用新 profile/run 重跑，不继承本次数值 PASS。
- 左右位置使用 XZ 面镜像；左右 wheel body frame 使用同一右手轴。determinant 为 `-1` 的空间反射不是 SO(3) 姿态，未被错误用作 rotation reference。

## Decision Gate Closure

- DG01 CLOSED：compiled mesh、轮轴、`0.05 m` nominal radius 和 Simulink `0.08 m` 差异已进入 profile/geometry manifest。
- DG02 CLOSED：nominal 连续分支、独立被动角关系、正反 continuation 和 210 样本 closure 已通过。
- DG03 CLOSED：`min singular >= 0.005`、`condition <= 40`；正式最坏值通过且无隐藏排除样本。
- DG04 CLOSED：解析 profile、MuJoCo 和重求被动角的中心有限差分三方通过，并由速度/虚功交叉验证。
- DG05 CLOSED：profile/run manifest 和非空目录拒绝覆盖已实测通过。
- DG06 CLOSED：所有正式阈值在 formal run 前写入 versioned config，正式 run 未放宽。

## Review Conclusion

Phase 15 在 simulation-only 边界内达到目标，无 blocking finding。允许创建 RECORD，并将 ROADMAP 状态更新为 `complete`。本 REVIEW 不关闭 Phase 05 或任何 MuJoCo–real/硬件 gate。

