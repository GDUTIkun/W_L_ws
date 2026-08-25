# Phase 16: Controller ↔ MuJoCo 确定性闭环运行基线 — RECORD

Status: `complete`

## Delivered Outcome

current nominal MuJoCo、canonical Controller Core 和 Adapter 已形成 ROS 无关的固定步数执行基线：每 10 ms 采样/控制一次，每条命令保持 5 个 2 ms physics steps，并可逐 tick 记录、显式 reset、故障注入、fresh/reset replay 和 SHA-256 留证。正式结果 `overall_pass=true`，没有使用真机数据。

## Frozen Technical Record

- 调度顺序为 `extractState → ControllerCore::step → acceptCommand → (writeControls → mj_step) × 5`；命令作用于 `[t_k,t_{k+1})`。
- source time 来自 MuJoCo；receipt time 是独立逻辑时钟；两者分列记录。
- episode reset 顺序为 MuJoCo/Adapter 后 Controller；旧 epoch 命令不可进入新 epoch。
- current Core 的六路 torque 和 nominal native ctrl 均为零；Phase 04 one-hot test 继续承担非零 mapping 证据。
- C++ executable 执行真实循环；Python 只编排、验证、比较与生成 manifest。
- 输出目录非覆盖；未来 SolidWorks revision、identified plant 和 Controller build 使用新 config/run，保留本次 nominal evidence。

## Evidence

- [自动验证记录](evidence/automated/2026-08-25-validation.md)
- [最终正式结果](evidence/automated/2026-08-25-nominal-v2/phase16_validation.json)
- [最终运行 manifest](evidence/automated/2026-08-25-nominal-v2/run_manifest.json)
- [Nominal fresh A](evidence/automated/2026-08-25-nominal-v2/nominal_a.csv)
- [Nominal fresh B](evidence/automated/2026-08-25-nominal-v2/nominal_b.csv)
- [Fault lifecycle log](evidence/automated/2026-08-25-nominal-v2/faults.csv)
- [Grounding](evidence/grounding.md)
- [复用与非覆盖契约](evidence/reuse_contract.md)
- [验证方法](../../../experiments/mujoco_controller_loop_validation.md)
- [REVIEW](REVIEW.md)

## Reproduction

```bash
cd /home/t/W_L_ws/ros_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-up-to wheel_leg_mujoco

cd /home/t/W_L_ws
./.venv/bin/python tools/experiments/run_mujoco_controller_loop.py \
  --profile nominal \
  --output-dir data/experiments/<new-phase16-run-id>/raw
```

已存在的正式 evidence 目录不可作为重跑目标。

## Limits and Next Use

- 本 PASS 只属于 current nominal、fixed-base、contact-disabled、zero Controller execution baseline。
- 下一 simulation-only 控制阶段可在同一 runner 上加入 production Joint PD/重力补偿，但必须建立新 Phase、新 Controller hash 和新 evidence。
- contact/floating-base、站立、WBC 与 NMPC 依照 ROADMAP 分层推进，不从本次 PASS 推断。
- 真机工作继续冻结；解冻并形成 identified profile 后，使用相同 runner/schema 追加重跑，不覆盖本记录。

