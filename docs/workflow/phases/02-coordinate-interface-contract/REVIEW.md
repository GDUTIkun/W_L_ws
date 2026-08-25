# Phase 02: 坐标系、单位、关节顺序与接口语义 — REVIEW

Status: `review`

## Review Scope

- PLAN：[`PLAN.md`](PLAN.md)
- 审查范围：Phase 02 当前工作树、用户人工证据、当前 `source.slx` 与 `wheel_leg.xml`
- 审查者与日期：Codex，2026-08-25

## Implementation Check

| PLAN Task | Delivered | Evidence | Result |
| --- | --- | --- | --- |
| T01 | 权威来源与 `R_A_from_B` 记号 | `PLAN.md`、coordinate contract | PASS |
| T02 | Simulink frame inspector 与当前 manifest | `inspect_simulink_frames.m`、`evidence/simulink_frame_*` | PASS |
| T03 | MuJoCo 静态/运行时审计 | 两个 audit 脚本与 manifests | PASS |
| T04 | FLU canonical、Simscape/MuJoCo/legacy 映射 | `docs/models/coordinate_frame_contract.md` | PASS |
| T05 | COM control site 与 joint mapping | `wheel_leg.xml`、`evidence/joint_coordinate_mapping.md` | PASS |
| T06 | MATLAB/Python 方向性测试 | `test_coordinate_frame_contract.m`、`test_mujoco_coordinate_contract.py` | PASS |
| T07 | 用户三视图/连接证据与后续真机 transfer | `USER_CHECKPOINT.md`、`evidence/manual/REVIEW.md` | PASS |
| T08 | 审查、记录和 ROADMAP 交接 | 本 REVIEW、RECORD、ROADMAP | PASS |

## Validation Results

| Validation | Command / Procedure | Actual Result | Evidence |
| --- | --- | --- | --- |
| MATLAB 坐标代数 | `matlab -batch "... test_coordinate_frame_contract"` | PASS；两个 proper rotations det=+1，legacy pack det=-1，positive yaw 与 wrench round-trip PASS | MATLAB stdout、测试源码 |
| MJCF 静态审计 | `tools/maintenance/audit_mujoco_frames.ps1` | 11 bodies、10 joints、6 sites、19 sensors、重复名称 0 | `evidence/mujoco_frame_manifest.json` |
| MuJoCo runtime 审计 | `conda run ... audit_mujoco_runtime.py` | MuJoCo 3.12.0 编译成功；nq=17、nv=16、nu=0；runtime probes PASS | `evidence/mujoco_runtime_manifest.json` |
| MuJoCo 坐标契约 | `conda run ... test_mujoco_coordinate_contract.py` | PASS；COM site、FLU、六 joint axes/sign、左右微扰、rolling、quaternion/yaw 通过 | 测试 stdout、测试源码 |
| 当前 Simulink manifest | `inspect_simulink_frames(...)` | 438 blocks，111 selected；批准的 LConn2–7 断线已记录 | `evidence/simulink_frame_manifest.json` |
| 当前 baseline smoke | `run_performance_smoke(5)` + assertions | exit 0；`simulationCompleted=true`、`controlStable=true`、QP feasible=1、NMPC status/fault=0 | MATLAB stdout；Phase execution note |
| 人工轴向核对 | Simscape/MuJoCo 三视图与 World-to-6DOF 连接 | world axes PASS；base/IMU origin 仅按实际证据判定 | `evidence/manual/REVIEW.md` |
| 文本检查 | `git diff --check` | PASS；仅行尾转换 warning | 命令 stdout |

## Findings

### Blocking

None.

### Non-blocking

1. 当前 MJCF compiled gravity 为零、`nu=0`、base 被 world weld；这是 Phase 04 基础模型的输入，不影响本 Phase 的坐标语义结论。
2. `base_control_frame` 表示当前 XML 质量分布算出的 nominal torso COM；真实质量/COM 必须在 Phase 07 标定。
3. Joint 相对符号已冻结，但逐关节 `b_joint` 需要 Phase 04 matching-pose + 第二姿态 FK；真实 encoder/torque 方向需要 Phase 06 复核。
4. `base_frame` 仍是 CAD-origin legacy sensor placeholder；真实 `imu_frame` 等安装 pose 确定后在 Phase 06 落地。

## Decision and Evidence Review

- 冻结决策是否被保持：是。Simulink baseline 保留内部 X前/Y上/Z右；统一边界采用 FLU；MuJoCo 保留局部 body/site 优势，转换位于 Adapter/接口边界。
- 证据是否足以支持技术结论：是。world/frame/rotation/joint-axis/sign/COM-site 结论均有源码、模型、数值测试或人工三视图支持；未获得真机证据的项目没有伪装为实验 PASS，而是转成交付给后续 Phase 的 gate。
- 是否存在需要新 Phase 的开放问题：Phase 04 joint offset/基础模型，Phase 06 真机 sensor/joint，Phase 07 质量与 COM；均已进入 ROADMAP，不阻塞本 Phase。

## Verdict

`PASS`
