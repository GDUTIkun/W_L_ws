# Phase 17: nominal Joint PD 与重力补偿 — PLAN

Status: `complete`

## Goal

在不连接真机、不启用轮地接触、不释放 floating base 的前提下，为 canonical Controller Core 实现可配置、可限幅、与 MuJoCo 解耦的六关节 Joint PD 和 current nominal 闭链重力补偿，并复用 Phase 16 的 2 ms physics / 10 ms control / 5-step ZOH runner，以真实仿真证据证明从单关节到双腿的保持、阶跃、对称性、限幅和扰动恢复，为后续 contact/floating-base Phase 提供经过验证的关节力矩层。

## Current State

- 已有：Phase 03 的 ROS 无关 `ControllerCore`、canonical `RobotState`/`TorqueCommand` 和 ROS wrapper；当前默认 Core 对有效状态只输出六路零力矩。
- 已有：Phase 04 的 Adapter 已冻结 `q_C=-q_M+b`、`dq_C=-dq_M`、`tau_M=-tau_C`、unit-gear actuator、watchdog 和 reset；unit gear 不是已标定真机执行器。
- 已有：Phase 14 已验证 current nominal MuJoCo 的重力势能梯度、质量矩阵、正逆动力学、闭链和耦合内部自洽；mass/COM/inertia 仍只是 nominal/CAD profile。
- 已有：Phase 15 已冻结每侧独立坐标 `[hip,knee,wheel]`、被动坐标 `[connect1,connect2]`、nominal 装配分支和约束降维映射 `S=[I;-pinv(Jp)Ja]`。
- 已有：Phase 16 已交付 fixed-base、contact-disabled、2 ms/10 ms/5-step 的 ROS 无关 runner、双时钟/reset/fault/replay、逐 tick CSV 和非覆盖 manifest。
- 对照：Simulink baseline 的下层控制使用加速度 PD、完整动力学/QP、接触和 floating-base load，不是本 Phase 可直接复制的纯 joint-torque PD；本 Phase 只复用其“先反馈、再模型补偿、最后限幅”的物理分层，不迁移 WBC/QP。
- 缺少：Core 还没有 joint reference、PD gains、gravity profile、torque clamp 或控制诊断；Phase 16 runner 仍把零输出写死为唯一 nominal 行为。
- Grounding：CBM generation `2026-08-25T06:16:31Z` 对 Phase 03/04 live symbols 有覆盖，但尚未跟踪 Phase 16 新 runner/scene；未跟踪文件和 `adapter.hpp:28` partial range 已直接读取。Graphify 只查询现有图，没有执行 extract/update。

## Scope

- Ground current Core/ROS wrapper、Phase 16 runner、Adapter、nominal model、Phase 14 gravity evidence、Phase 15 reduced-coordinate contract 和 Simulink baseline，冻结迁移边界。
- 定义 Core 内部 `JointReference`，支持配置初始静态参考和由 in-process runner 在明确 control tick 切换的分段常值参考；本 Phase 不建立新 ROS reference message。
- 扩展 `ControllerConfig`，显式选择 `zero` 或 `joint_pd_gravity` mode，并配置六路 `q_ref/dq_ref/Kp/Kd/torque_limit`、gravity profile 和 enable flags；默认 mode 保持 `zero`。
- 实现 canonical joint-space 控制律、逐关节对称力矩限幅和可审计 diagnostics：

  ```text
  tau_pd = Kp * (q_ref - q) + Kd * (dq_ref - dq)
  tau_raw = tau_pd + tau_g(q, q_n_from_b)
  tau_cmd = clamp(tau_raw, -tau_limit, +tau_limit)
  ```

- 为 current nominal 双腿闭链建立 Controller-side 独立刚体重力 profile：由 versioned body mass/COM、固定 transforms、canonical joint mapping、nominal passive branch 和 reduced coordinates 计算 `tau_g`；Core/ROS package 不链接 MuJoCo，不在运行时读取 `qfrc_bias`。
- 用 MuJoCo reduced gravity 与势能有限差分作为离线双重 oracle，验证 Controller gravity profile 的方向、顺序和数值，不以闭环“看起来稳定”代替模型验证。
- 向 Phase 16 C++ runner 追加 controller profile、reference tick、扰动力矩和 diagnostics 能力；保留旧 zero/fault 场景与列语义，新的 Phase 17 Python wrapper 只负责编排、oracle、指标、manifest 和比较。
- 使用 exploratory tuning run 选择左右共享的 hip/knee/wheel gains 与 simulation-only torque limits；正式阈值必须在 formal holdout run 前冻结到 versioned config，探索结果与正式 evidence 分目录保存。
- 依次验证 gravity-only 静态平衡、PD-only、PD+gravity、左右各 joint class 小阶跃、双腿对称阶跃、正负方向、限幅、外部扰动恢复、reset replay 和跨进程确定性。
- 回归 Phase 02/04/14/15/16，并固化 model/controller/config/runner/output hash 和非覆盖复用入口。

## Out of Scope

- 真机上电、STM32/树莓派联调、Hardware Adapter、传感器采集、Load Cell、执行器辨识或任何 MuJoCo–real 结论。
- 真实电机 torque/current/velocity/temperature limit、减速比、deadzone、friction、armature、delay 或 driver safety；本 Phase 的 torque clamp 只属于 nominal ideal-actuator 仿真保护与控制律验证。
- 轮地接触、摩擦、滑移、接触力、floating base、站立、base height/pitch、wheel position、平衡或抗跌倒结论。
- Integral action、anti-windup、gain scheduling、trajectory generator、planner、inverse dynamics、computed torque、QP/WBC、NMPC 或 Simulink 全算法迁移。
- 新 ROS topic/message/service、在线动态参数更新或 reference source arbitration；ROS 只加载静态 controller profile 并做兼容性 smoke。
- 修改 canonical joint order、frame/sign/offset、RobotState/TorqueCommand schema、Adapter mapping、Phase 16 timing 或历史 evidence。
- 用 MuJoCo API、compiled model pointer、runtime `qfrc_bias` 或查表插值作为 production Core 的隐藏依赖。

## Frozen Decisions

- 正式 plant 继续使用 `phase16_contact_free.xml`：current nominal、`base_weld` 开启、双腿 closure 开启、contact 全局关闭；Phase 17 PASS 不外推到 contact/floating-base。
- physics/control timing 保持 `0.002 s / 0.010 s / 5-step ZOH`，每个 reference event 在 tick `t_k` 的 state sample 前生效，产生的命令作用于 `[t_k,t_{k+1})`。
- Core 默认配置继续是 `zero` mode，保证 Phase 03/16 的安全零输出与既有 ROS launch 默认行为不变。只有显式选择并通过完整配置校验的 `joint_pd_gravity` profile 才能输出非零力矩。
- PD 使用 canonical measured q/dq，不数值微分，不加滤波、不用积分：`Kp(q_ref-q)+Kd(dq_ref-dq)`；六路 gains 非负且有限，左右同类关节默认共享 gains，但配置和日志仍保存完整六路数组。
- `JointReference` 只含六路 `q_ref/dq_ref`。runner 支持分段常值 reference；Core `reset()` 恢复配置的初始 reference 并清除 sample history，旧 episode reference 不跨 reset。
- 重力补偿是 `tau_g=∂U/∂q_C` 的 canonical reduced generalized torque，包含闭链所有 nominal rigid bodies；wheel spin 重力项按真实 compiled COM offset 计算，不能为追求理想零值而丢弃小偏心。计算允许使用 base quaternion 把 world gravity 表达到 base frame，但本 Phase 只验 fixed-base identity pose。
- Controller-side gravity evaluator只依赖普通 C++17 和 versioned coefficients/rigid-body profile。MuJoCo reduced bias 与势能差分只存在于验证工具，不进入 `wheel_leg_core`、ROS wrapper 或 Controller runtime。
- 力矩限幅逐关节、对称、在 PD+gravity 求和后执行；不加入 rate limit。saturation flag、`tau_pd`、`tau_g`、`tau_raw`、`tau_cmd` 必须逐 tick 记录，不能只保存最终 command。
- gain/limit tuning 使用明确 exploratory case set；formal case set 至少包含未参与选择的幅值/方向组合。formal run 开始后不得放宽 thresholds，失败只能修实现、建立新 config/run 或 REVIEW=REWORK。
- 初次关节阶跃由 reference schedule 产生，plant 从有效 qpos0/closure branch reset，不直接写入不满足闭链的被动 qpos。
- Phase 16 runner 原地扩展而不复制第二套物理循环；旧 `phase16_nominal.json`、zero/fault wrapper 和正式 evidence 保留。新增列只允许追加，旧字段名称/语义不得改变。
- 所有新 run 使用新目录；current nominal、未来 SolidWorks revision 和 identified profile 都有独立 gravity/controller config 与 manifest，模型变化不继承 gains、gravity coefficients 或 PASS。

## Open Questions / Decision Gates

- **DG01 / CLOSED / CODEX_DECISION — 控制层级：** 本 Phase 只实现 joint-torque PD+gravity，不移植 Simulink 的 acceleration-QP/WBC；后者保留给后续独立 Phase。
- **DG02 / CLOSED / CODEX_DECISION — gravity authority：** production Core 使用独立 reduced rigid-body gravity evaluator；MuJoCo reduced bias 与势能差分是两个验证 oracle，不能作为 Controller 输入。
- **DG03 / CLOSED / CODEX_DECISION — reference contract：** 本阶段使用 Core 内部 `JointReference` 与 runner 分段常值 schedule，不新增公共 ROS reference schema；动态 planner 接口以后单独冻结。
- **DG04 / OPEN / EVIDENCE — gravity profile：** current nominal mass/COM/transforms、passive branch、canonical sign 和 coefficient schema 必须完成 provenance manifest，并在冻结工作域内通过双 oracle 后才能启用 gravity compensation。
- **DG05 / OPEN / EVIDENCE — gains and limits：** exploratory tuning 必须给出稳定且左右一致的 hip/knee/wheel gains、simulation-only torque limits 和选择理由；在 formal holdout 前冻结。
- **DG06 / OPEN / EVIDENCE — closed-loop gates：** gravity-only、PD-only、PD+gravity、阶跃、对称、饱和和扰动场景的 error/settling/overshoot/velocity/torque/finite 指标必须全部通过预冻结阈值。
- **DG07 / OPEN / EVIDENCE — backward compatibility：** Phase 16 zero/fault validation、Adapter/Core tests、ROS static zero profile、Phase 14/15 runners必须在新 Core/runner 上继续 PASS。
- **DG08 / CLOSED / SCOPE — real/contact claims：** 本 Phase 不关闭执行器、contact、floating-base、站立或真机 gate；ideal actuator 下选出的 gains/limits 不能直接部署到真机。

## Interfaces and Compatibility

- 输入：canonical `RobotState`；versioned controller/gravity profile；内部 `JointReference`；2 ms/10 ms timing；reference/disturbance schedule；current nominal model revision。
- 输出：canonical `TorqueCommand`，顺序固定为 `[left_hip,left_knee,left_wheel,right_hip,right_knee,right_wheel]`，单位 N·m；同 source timestamp；逐 tick PD/gravity/raw/clamped/saturation diagnostics。
- Gravity oracle：MuJoCo full generalized bias 按 Phase 15 `S` 投影到 driven reduced coordinates，再按 Phase 02/04 canonical sign 转换；独立势能中心差分作为第二参考。
- ROS profile：node 只声明/读取静态 mode、reference、gains、limits 与 gravity coefficients；参数非法时启动失败，不退回非零默认值。
- 必须保持：Core 无 ROS/MuJoCo 依赖；TorqueCommand 仍是 desired canonical output-axis torque；Adapter watchdog/sign/order/reset 不变；default zero mode；Phase 16 timing/log 基础列和非覆盖语义。
- 允许改变：`ControllerConfig/StepResult` 增加控制配置与 diagnostics；Core 增加 reference setter/gravity evaluator；ROS node 增加静态参数；Phase 16 runner 追加 profile/schedule/disturbance/diagnostic 参数和列；新增 Phase 17 config/wrapper/tests/docs/evidence。

## Tasks

| ID | Task | Input | Deliverable | Validation | Status |
| --- | --- | --- | --- | --- | --- |
| T01 | Ground live Core↔ROS↔runner↔Adapter 链、Simulink 对照和 Phase 14/15 model contracts | CBM/source、Phase 14–16 RECORD/evidence、Simulink baseline | grounding、职责/依赖/增量边界、DG01–DG03 关闭记录 | 每个职责映射到当前符号/文件；不把 WBC、contact 或 MuJoCo oracle 放进 Core | done |
| T02 | 推导并冻结 current nominal reduced gravity profile，关闭 DG04 | compiled mass/COM/transforms、Phase 15 branch/S、canonical mapping | `phase17_nominal.json` gravity section、provenance/coefficients manifest、独立 evaluator contract | 版本/hash/单位/frame/sign 完整；wheel 小偏心项未被静默删除；双 oracle sweep 通过预冻结误差 | done |
| T03 | 扩展 Controller Core 的 mode、reference、PD、gravity、clamp 与 diagnostics | T01/T02、现有 Core contract | 默认 zero mode、显式 PD+gravity mode、reference/reset/config validation、逐项 diagnostics | 普通 C++ tests 覆盖公式、sign/order、非法 config/reference、reset、saturation、determinism | done |
| T04 | 为 ROS wrapper 增加静态 controller profile | T03、现有 ControllerNode | ROS params/YAML、严格启动校验、zero 与 PD profile launch 入口 | 默认 launch 仍零输出；非法参数启动失败；PD profile 输出字段有限且同 timestamp | done |
| T05 | 兼容扩展 Phase 16 C++ runner 与建立 Phase 17 wrapper | Phase 16 runner/schema、T03、reference/disturbance case config | control scenario、schedule/disturbance/diagnostic 日志、`run_mujoco_joint_pd_gravity.py`、manifest | 旧列语义不变；非空目录拒绝；物理循环不在 Python 重写；Phase 16 wrapper仍可解析 | done |
| T06 | 执行 exploratory gain/limit tuning 并关闭 DG05 | T02–T05、冻结 tuning cases | 左右共享 hip/knee/wheel gains、limits、选择记录与 formal holdout config | 所有 tuning run 追加保存；formal thresholds/config 在 formal run 前 hash 冻结 | done |
| T07 | 正式验证 gravity model 与静态补偿 | T02/T06、Phase 15 workspace/holdout poses | gravity sweep、gravity-only 短时平衡、PD-only 对照、最差姿态/关节记录 | 双 oracle、canonical signs、有限值、静态加速度/漂移和重力补偿改善全部过阈值 | done |
| T08 | 正式验证 joint closed loop 并关闭 DG06 | frozen profile、holdout reference/disturbance matrix | 单关节/双腿、正负阶跃、PD+G、clamp、disturbance、symmetry、replay CSV/summary | error 收敛、settling/overshoot/velocity/torque/saturation/recovery/finite/determinism 全部过阈值；失败样本不删除 | done |
| T09 | 执行 backward compatibility 和历史回归，关闭 DG07 | T03–T08、Phase 02/04/14/15/16 入口 | colcon tests、coordinate、Adapter、Phase 14/15、Phase 16 zero/fault、ROS reset smoke | 全部 PASS；Phase 16 正式 evidence 与旧 configs 未覆盖；默认 zero 行为保持 | done |
| T10 | 固化方法、复用契约、正式 evidence 并准备 REVIEW | T01–T09 | 方法文档、README、grounding、automated evidence、Execution Notes、REVIEW 输入 | DG01–DG08 全关闭；manifest 可跨 nominal/revision/identified 解析；无 contact/real 夸大结论 | done |

任务状态只使用 `todo / doing / done / blocked`。

## Validation Plan

### Automated

- `cd ros_ws && source /opt/ros/jazzy/setup.bash && colcon build --symlink-install --packages-up-to wheel_leg_mujoco`：Core、ROS wrapper、Adapter 和扩展 runner 在 C++17/Jazzy/MuJoCo 3.7.0 下构建。
- `cd ros_ws && source /opt/ros/jazzy/setup.bash && colcon test --packages-select wheel_leg_core wheel_leg_ros wheel_leg_mujoco && colcon test-result --verbose`：zero compatibility、PD algebra、gravity、reference/reset、clamp、ROS 和 Adapter tests 无失败。
- `./.venv/bin/python tools/experiments/run_mujoco_joint_pd_gravity.py --config simulation/mujoco/config/phase17_nominal.json --output-dir data/experiments/<new-phase17-run-id>/raw`：运行正式 gravity/hold/step/limit/disturbance/replay matrix，全部 gate PASS。
- 对同一 frozen config 使用两个新 run ID：规范化输入、CSV/summary 和最差数值 diff 满足 deterministic threshold；不能复用非空目录。
- `./.venv/bin/python tools/experiments/run_mujoco_controller_loop.py --profile nominal --output-dir data/experiments/<new-phase16-regression-id>/raw`：Phase 16 zero/fault 的 24 gates 继续 PASS。
- `./.venv/bin/python tools/maintenance/test_mujoco_coordinate_contract.py`：canonical frame/order/sign/offset 回归 PASS。
- `./.venv/bin/python tools/experiments/run_mujoco_internal_dynamics.py --output-dir data/experiments/<new-phase14-regression-id>/raw`：Phase 14 九项内部动力学回归 PASS。
- `./.venv/bin/python tools/experiments/run_mujoco_closed_chain_kinematics.py --output-dir data/experiments/<new-phase15-regression-id>/raw`：Phase 15 闭链/reduced Jacobian 回归 PASS。
- 独立 ROS Domain 启动 default zero 与显式 Phase 17 profile：验证参数、topic、timestamp、reset 与有限/限幅 torque；ROS wall-clock jitter 不进入闭环性能 verdict。

### Manual / Evidence

- Codex 在 tuning 前审查 gravity profile provenance、canonical sign、被动 branch 和 reduced projection；Core 不得包含 MuJoCo include/link/runtime data。
- tuning 与 formal case ID、reference amplitude/direction、duration、disturbance pulse、gains、limits、seed、thresholds 和 config hash 全部进入 manifest；formal 失败样本不得隐藏或改阈值后覆盖重跑。
- 正式 REVIEW 逐 joint class 检查 `q_ref/q/dq/tau_pd/tau_g/tau_raw/tau_cmd/saturated`，确认 torque clamp 位于补偿求和后且左右无特例符号。
- 比较 zero、gravity-only、PD-only 和 PD+gravity，结论限定为冻结工况；“PD+gravity 改善”必须由 steady-state error/静态漂移指标支持，不能只看动画。
- 检查 Phase 16、Phase 14/15 历史 evidence 未修改，新 runner/config/schema 能同时解析旧 nominal 与未来 identified profile。

## Acceptance Criteria

- [x] T01–T10 完成，DG01–DG08 全部关闭且没有未记录偏差。
- [x] 默认 Controller 配置仍严格零输出；只有显式有效的 `joint_pd_gravity` profile 能产生非零 command，非法配置/reference fail closed。
- [x] Core 内实现冻结 PD 公式、独立 reduced gravity compensation 和求和后逐关节对称 clamp；不依赖 ROS、MuJoCo 或 runtime plant oracle。
- [x] current nominal gravity profile 的 mass/COM/frame/branch/sign/hash provenance 完整，并在冻结工作域通过 MuJoCo reduced bias 与势能差分双 oracle。
- [x] fixed-base/contact-disabled 正式矩阵覆盖 hip/knee/wheel、左右、正负方向、单关节/双腿、保持/阶跃/饱和/扰动/reset；所有样本有限，joint velocity 有明确上界并通过预冻结指标。
- [x] PD+gravity 相对 PD-only 的静态保持误差/漂移有量化改善；不据此声明接触、站立、floating-base 或真机稳定。
- [x] torque command 不超过 profile limit，饱和 tick 可审计；limit 明确标记为 simulation-only，不映射为真实电机安全值。
- [x] Phase 16 的 2 ms/10 ms/5-step、双时钟、fault/reset/replay、旧列语义和非覆盖规则保持；zero/fault regression PASS。
- [x] coordinate、Core/ROS/Adapter、Phase 14、Phase 15、Phase 16 全部回归 PASS；历史正式 evidence 未覆盖。
- [x] 新 gravity/controller/model revision 可通过 profile/hash/new run 切换，不修改控制算法；方法、README、ROADMAP、实现和真实 evidence 一致。

## Execution Notes

- 2026-08-25：用户要求制定 Phase 17，不开始实现。Phase 仅覆盖 current nominal、fixed-base、contact-disabled Joint PD+gravity；真机、contact、floating-base 和站立继续冻结/后移。
- 2026-08-25：采用 ponytail 最小控制层：静态/分段常值 joint reference、measured-velocity PD、独立 gravity feedforward、magnitude clamp；不加入积分、滤波、rate limit、trajectory planner、QP 或新 ROS message。
- 2026-08-25：Graphify 只查询现有本地图核对 Phase 16、Simulink baseline、gravity/coordinate 历史关系，没有执行 extract/update。live code 以 CBM generation `2026-08-25T06:16:31Z` 和直接源码读取为准。
- 2026-08-25：current nominal 重力被约化为每腿 3 个解析谐波项；150 个闭链姿态对 MuJoCo reduced bias 与闭链势能中心差分的最差误差分别为 `5.27e-12 N·m`、`6.78e-9 N·m`，轮体偏心项最大 `4.07e-4 N·m` 并保留。
- 2026-08-25：探索调参冻结为 hip/knee `Kp=12, Kd=1.5`，wheel `Kp=0.3, Kd=0.05`，simulation-only limits 为 `[6,6,1] N·m` 每侧；补齐 C++/JSON 交叉检查和 revision profile 注入后的正式 `formal-v3` 全 gate PASS。

## Blockers

None. 本 Phase 不依赖真机、STM32、Load Cell、Hardware Adapter、identified actuator、轮地接触或 floating-base；DG04–DG07 由本 Phase 自身仿真与正式 evidence 关闭。
