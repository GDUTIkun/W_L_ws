# Phase 17: nominal Joint PD 与重力补偿 — RECORD

Status: `complete`

## Outcome

canonical Controller Core 已交付默认安全零输出与显式 opt-in Joint PD+current nominal 解析重力补偿；在 2 ms physics / 10 ms control / 5-step ZOH、fixed-base/contact-disabled MuJoCo 中完成双 oracle、保持、正负阶跃、对称、限幅、扰动和确定性验证，最终 `formal-v3` 全 PASS。

## Delivered

- 通用 `ControllerConfig` gravity profile、JointReference、PD/gravity/clamp 与 diagnostics。
- ROS `current_nominal`/`configured_harmonics` 静态 profile 和 YAML。
- Phase 16 runner 的兼容 control scenario，以及 profile/reference/disturbance/diagnostic 追加列。
- `phase17_nominal.json`、`run_mujoco_joint_pd_gravity.py`、验证方法、grounding、复用契约和非覆盖 evidence。

## Verification Evidence

- [正式 validation](evidence/automated/2026-08-25-formal-v3/phase17_validation.json)：14/14 gates，`overall_pass=true`。
- [正式 manifest](evidence/automated/2026-08-25-formal-v3/run_manifest.json)：9 inputs、25 outputs，最终 hash 复核匹配。
- [自动验证记录](evidence/automated/2026-08-25-validation.md)。
- [历史回归](evidence/automated/2026-08-25-regression/)：Phase 14/15 PASS，最新 `phase16-v2` 24/24 gates PASS。
- Jazzy build PASS；19 tests、0 errors/failures/skipped；coordinate contract PASS。
- [REVIEW](REVIEW.md)：`PASS`，无 blocking finding。

## Decisions Confirmed

- 控制律固定为 measured-state Joint PD + analytic gravity feedforward + post-sum symmetric clamp；不加入积分、滤波、rate limit、QP/WBC 或 planner。
- current nominal gains：hip/knee `Kp=12, Kd=1.5`；wheel `Kp=0.3, Kd=0.05`。
- current nominal gravity 每腿三项解析谐波，包含 wheel COM 微偏心；MuJoCo bias/势能只作为离线 oracle。
- 默认 mode 永远为 zero；非零控制必须显式、完整、合法配置。
- 新模型 revision 使用新 profile/config/run，不改控制算法、不覆盖本次证据。

## Deviations from PLAN

- 独立 gravity evaluator 最终采用 versioned analytic harmonic coefficients，而不是在 Core 内保留完整刚体树；这是 PLAN 允许的 coefficient profile 路径，并由 150 姿态双 oracle 验证。
- 没有新增 ROS reference message 或专用 launch；静态参数/YAML 与现有 node 入口完成本 Phase contract。

## Known Limitations and Follow-ups

- 只证明 current nominal、fixed-base identity pose、contact-disabled ideal-actuator simulation；不直接部署真机。
- 下一建议阶段是 ROADMAP 中的 nominal 轮地接触与 floating-base plant 验证；先验证 plant/contact/base state，不提前做站立。
- SolidWorks revision 后重新导出时建立新模型/profile，重跑 Adapter、Phase 14/15、gravity oracle 和本 Phase matrix。
- 真机工作继续冻结；解冻后先完成执行器/通信/传感器/安全 gate，再进行 MuJoCo–real 辨识和 identified profile 分层复现。

## Key Links

- [PLAN](PLAN.md)
- [REVIEW](REVIEW.md)
- [Grounding](evidence/grounding.md)
- [复用契约](evidence/reuse_contract.md)
- [验证方法](../../../experiments/mujoco_joint_pd_gravity_validation.md)
- [ROADMAP](../../ROADMAP.md)
