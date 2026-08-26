# Phase 19 预冻结探索验证记录（2026-08-26）

结论：`REWORK`。本记录是允许失败的 pre-freeze decision gate，不是 formal standing evidence，不含真机数据，也不批准 Controller Core standing mode。

## Authority 与入口

- profile：[`phase19_exploration.json`](../../../../../../simulation/mujoco/config/phase19_exploration.json)
- scene：[`phase19_standing.xml`](../../../../../../simulation/mujoco/model/phase19_standing.xml)
- runner：[`run_mujoco_simple_standing_exploration.py`](../../../../../../tools/experiments/run_mujoco_simple_standing_exploration.py)
- primary output：[`2026-08-26-prefreeze/summary.json`](2026-08-26-prefreeze/summary.json)
- fresh-process replay：[`2026-08-26-prefreeze-replay/summary.json`](2026-08-26-prefreeze-replay/summary.json)

runner 直接读取完整 MuJoCo/contact plant，只用于判断四状态/common-wheel 结构是否值得进入 C++ 实现。它不是第二套生产 Controller↔Adapter loop；PLAN 明确要求 pre-freeze 失败时停在 Core 实现之前。

## Commands and Actual Results

```bash
./.venv/bin/python tools/experiments/run_mujoco_simple_standing_exploration.py \
  --output-dir docs/workflow/phases/19-nominal-planar-simple-standing/evidence/exploratory/2026-08-26-prefreeze
```

实际：脚本生成完整输出并以非零状态结束；`decision=REWORK`、`overall_pass=false`。

```bash
./.venv/bin/python tools/experiments/run_mujoco_simple_standing_exploration.py \
  --output-dir docs/workflow/phases/19-nominal-planar-simple-standing/evidence/exploratory/2026-08-26-prefreeze-replay
```

实际：fresh-process replay 得到完全相同的 `timeseries.csv` 与 `summary.json`：

```text
timeseries.csv  59004623b897a760d414d4701826008159ff7912d9061b66bb3fa40da5e48932
summary.json     3deafea120f73808a92f71d25824069c1eab2b4ee6b9ee26b47ffe4835b52e3c
```

再次指向非空 replay 目录时，在仿真前拒绝覆盖：

```text
Refusing to overwrite non-empty output directory: .../2026-08-26-prefreeze-replay
```

辅助检查：

```bash
./.venv/bin/python -m py_compile tools/experiments/run_mujoco_simple_standing_exploration.py
./.venv/bin/python tools/maintenance/test_mujoco_coordinate_contract.py
git diff --check
```

实际：全部 PASS。Phase 18 与 Phase 19 scene 编译维度均为 `nq=17, nv=16, nu=6, neq=3, ngeom=74, timestep=0.002`。

## Key Evidence

### Local model

- state：`[base_control_frame x error, vx, pitch, world omega_y]`
- input：左右相等的 common wheel torque；native/canonical 符号在 summary 中显式记录
- sample：`10 ms`，底层仍为 `2 ms × 5-step ZOH`
- controllability rank：`4`
- closed-loop poles：`0.5430`、`0.9650`、`1.0320 ± 0.00985j`
- spectral radius：`1.0320567369`，门槛 `< 1`
- nominal one-tick pitch-rate drift：`0.0863837 rad/s`

因此“可控”不等于“当前 gain 已稳定”；找到的 10 秒候选是带偏差的非线性有界运动，不能从动画或有限时间未倒推出局部渐近稳定。

### Full-plant 10 s cases

| Case | Final `|x|` | Final `|pitch|` | Lateral max | Roll/Yaw max | Both-contact fraction | Result |
| --- | ---: | ---: | ---: | ---: | ---: | --- |
| unconstrained nominal | `0.00829 m` | `0.00994 rad` | `0.0544 m` | `0.1312 rad` | `0.999` | FAIL: 3D leakage |
| diagnostic stabilization nominal | `0.02617 m` | `0.03295 rad` | `0.0216 m` | `0.1553 rad` | `0.999` | FAIL |
| diagnostic stabilization pitch `+0.01` | `0.02706 m` | `0.03528 rad` | `0.0214 m` | `0.1394 rad` | `0.998` | FAIL |
| diagnostic stabilization pitch `-0.01` | `0.02615 m` | `0.03291 rad` | `0.0220 m` | `0.1190 rad` | `1.000` | FAIL |

“diagnostic stabilization”只施加 lateral position/rate 与 roll/yaw rate 外力，用来隔离 sagittal 问题。它不在冻结 scope 内，不能提供 PASS；而且即便有该辅助，pitch/x recovery 仍失败。

## Review Implication

当前设计把两个互斥前提放在一起：一方面只允许 sagittal/common-mode controller，另一方面又要求完整 3D floating plant 的 roll/yaw/Y 泄漏过硬门槛。下一轮必须先选择并明确实现：

1. 显式受约束、可审计的 2D sagittal validation plant，用于验证简单站立；完整 3D standing 另立后续层；或
2. 保持完整 3D plant，并扩展 controller 对 lateral/roll/yaw 的控制 authority，这将超出当前 common-wheel 四状态范围。

在该边界重划、零轮扭矩 equilibrium 和稳定 gain 重新通过 DG02–DG05 前，不进入 C++ Core、formal matrix、RECORD 或 ROADMAP complete。
