# Simulink MPC–WM-WBC baseline

本目录是迁移到 C++ Controller Core 和 MuJoCo 时的只读行为对照基线。它不是 current runtime、示例模型或 terrain adaptation 完成版；它冻结的是截至 2026-08-24 已在平地验证过的三维 Simscape + 16-state NMPC + 12-DoF weighted WM-WBC 控制链。

第一次接手时，先阅读：

1. 本 README：如何启动、验证、生成输出；
2. ../../docs/models/simulink_mpc_wm_wbc_baseline.md：坐标、状态、输入、模型公式、接口和能力边界；
3. SNAPSHOT_MANIFEST.md：来源版本、复制边界和关键文件哈希。

## 1. 快照身份

| 项目 | 值 |
| --- | --- |
| 源快照 | D:\Workspace\CodeWorkspace\model\simulate\proformance_test |
| 源仓库提交 | acb36ec229354142f4c77bb57073e8f590c418fb |
| 迁入日期 | 2026-08-24 |
| 已验证 MATLAB release | R2024b |
| 主模型 | model/simulate/two_legs/source.slx |
| 降阶对照模型 | model/simulate/two_legs/source_common.slx |
| 初始化 | model/simulate/two_legs/startup.m |
| 顶层入口 | open_proformance_test.m |
| 短 smoke | run_performance_smoke.m |
| NMPC build tag | paper_eq12_v1 |
| NMPC dynamicsVersion | 7 |

源目录在复制时对本项目路径无未提交改动。主模型和关键控制文件在复制后做了 SHA-256 对照，结果见 SNAPSHOT_MANIFEST.md。

## 2. 目录边界

    simulink_baseline/
    ├── model/
    │   ├── code/                         # 运动学/降阶控制与历史对照代码
    │   └── simulate/two_legs/
    │       ├── source.slx                # 权威三维 Simscape 模型
    │       ├── source_common.slx         # 降阶/共模对照模型
    │       ├── startup.m                 # 参数与默认控制配置
    │       ├── spatial_two_leg_qp_core.m # 12-DoF WM-WBC
    │       ├── full_base_*.m             # 16-state NMPC 模型/OCP/信号链
    │       └── generated/                # full solver runtime + common solver source bundle
    ├── calibration/studies/
    │   ├── 2026_08_stage1_performance/   # 平地直线/外扰/差模回归入口
    │   └── 2026_08_two_leg_model_tests/  # 转向与大 yaw 回归入口
    ├── evidence/                         # 小型、受控的既有结果摘要
    ├── resources/                        # MATLAB Project 元数据
    ├── open_proformance_test.m
    ├── run_performance_smoke.m
    ├── Proformance_test.prj
    └── SNAPSHOT_MANIFEST.md

未复制的内容：

- slprj、work、source.slxc、CMake object/cache；
- 历史 raw MAT、完整 timeseries、批量权重扫描和图片；
- 与当前默认配置无关的十余套旧 Acados solver 变体；
- 第三方 Acados、CasADi 仓库；
- MuJoCo 或 ROS2 runtime 代码。

generated 下在本机保留当前 full 16-state solver 的冻结 runtime，以及 optional 8-state common-mode solver 的生成源码 bundle。后者当前没有顶层 S-Function，因此 startup 会明确显示 direct 8-state solver not built；主模型 source.slx 使用的是可用的 full solver。两者都不是权威模型源码，且由 generated/.gitignore 排除在版本控制之外；权威定义仍是 startup.m、full_base_body_dynamics.m、full_base_wheel_state_space.m、full_base_nmpc_ocp.m 和 build_base_nmpc_solver.m。

## 3. 软件依赖

运行基线需要：

- MATLAB R2024b（本快照验证版本）；
- Simulink；
- Simscape 与 Simscape Multibody；
- Optimization Toolbox（quadprog）；
- Control System Toolbox；
- Windows x64，与快照内 mexw64/DLL ABI 兼容。

重新生成 NMPC solver 还需要外部 Acados 与 CasADi。它们不复制到本目录，原因是第三方依赖不属于模型源码。build_base_nmpc_solver.m 默认在仓库根目录寻找：

    tools/acados
    tools/casadi

当前 W_L_ws 尚未自带这两个依赖。若冻结 MEX 在另一主机或 MATLAB release 下不可加载，应先提供受控版本的外部依赖并重新构建，不要用旧二进制错误判断控制器模型失效。

## 4. 首次启动

推荐从 MATLAB 命令窗口运行：

    baselineRoot = 'D:\Workspace\W_L_ws\simulation\simulink_baseline';
    cd(baselineRoot);
    project = matlab.project.loadProject(baselineRoot);
    context = open_proformance_test(true);

open_proformance_test 会：

1. 用自身文件位置解析根目录，不依赖原 CodeWorkspace 绝对路径；
2. 把 model/code（仅保留当前运行依赖）、model/simulate/two_legs 和核心测试目录放到 MATLAB path 前部；
3. 把 Simulink cache/codegen 输出重定向到本目录 work/；
4. 运行 startup.m；
5. 检查 differential_leg_force_stabilizer 实际解析路径仍在本快照内；
6. 加载并按需打开 source.slx。

`model/code` 当前只保留 `differential_drift_stabilizer.m` 和
`differential_leg_force_stabilizer.m`。两者均由当前
`spatial_two_leg_qp_core` 直接调用；baseline 默认配置虽关闭其修正量，
但函数仍是模型解析和执行路径的一部分。旧 2D/3D 符号推导、平面降阶核对
和历史 helper 单元测试不属于可运行 baseline，未收入本快照。

若只需非交互加载：

    context = open_proformance_test(false);

不要从其他工作区已经污染的 MATLAB session 中直接运行 generic 名称 source.slx。若出现同名模型或函数冲突，关闭所有模型、恢复默认 path 后重新执行上述入口。

## 5. 快速验证

### 5.1 静态加载与模型 update

    context = open_proformance_test(false);
    assert(startsWith(string(which('spatial_two_leg_qp_core')), string(context.root)));
    set_param('source', 'SimulationCommand', 'update');

预期：

- startup 显示 full 16-state/12-input NMPC S-Function ready；optional direct 8-state solver 可以显示 not built；
- spatial_two_leg_qp_core 和 anti-split 相关函数解析在本目录；
- source 完成 update，无 missing block、missing variable 或 solver load 错误。

### 5.2 五秒 Accelerator smoke

    summary = run_performance_smoke(5);

通过条件：

- simulationCompleted = true；
- controlStable = true；
- QP feasible ratio = 1；
- nmpcStatusMax = 0；
- nmpcFaultRatio = 0。

该 smoke 只验证迁入后全链路可执行，不等价于重新完成 1 m/s 性能验收。

### 5.3 单元/结构契约

在 open_proformance_test(false) 后运行：

    test_paper_wheel_relative_dynamics
    test_wheel_position_coordinate_contract
    test_wheel_contact_pfaffian_contract
    test_coupled_two_leg_qp
    test_paper_hierarchical_wbc_contract

修改状态顺序、wheel-position 定义、interaction-wrench 顺序、contact basis 或 WBC Jacobian 后，应先跑这些快速测试，再启动长时间 Simscape 仿真。

## 6. 平地性能入口

所有长工况默认关闭动画，以避免渲染开销污染 timing。

### 6.1 1 m/s 启动—匀速—制动

    context = open_proformance_test(false);
    studyDir = fullfile(context.root, 'calibration', 'studies', ...
        '2026_08_stage1_performance');
    addpath(studyDir, '-begin');
    cd(studyDir);
    summary = run_stage1_straight_cases(1.0, 20, 2.0);

既有通过证据位于 evidence/flat_1ms_start_cruise_brake_summary.csv。

### 6.2 1 m/s、0.2 rad/s、90° 左转

    context = open_proformance_test(false);
    studyDir = fullfile(context.root, 'calibration', 'studies', ...
        '2026_08_two_leg_model_tests');
    addpath(studyDir, '-begin');
    cd(studyDir);
    summary = test_large_yaw_turning_simulink( ...
        "HS2_90_left_v100_yaw020", struct(), inf, struct(), false);

既有通过证据位于 evidence/flat_1ms_turning_summary.csv。

### 6.3 低速 360° 与动画

    context = open_proformance_test(false);
    studyDir = fullfile(context.root, 'calibration', 'studies', ...
        '2026_08_two_leg_model_tests');
    addpath(studyDir, '-begin');
    cd(studyDir);

    % 性能运行
    summary = test_large_yaw_turning_simulink( ...
        "C1_360_left_continuous", struct(), inf, struct(), false);

    % 目视运行：第五个参数打开 Mechanics Explorer
    summaryVisual = test_large_yaw_turning_simulink( ...
        "C1_360_left_continuous", struct(), inf, struct(), true);

低速 360° 已物理稳定完成，但历史严格门限因 wrench/contact residual 未全部通过，不能写成完整验收 PASS。

## 7. 已有平地证据

以下数字来自迁入前已经保存的实验摘要，本次复制不会把它们重新解释成目标路径重测结果。

| 工况 | 结果摘要 |
| --- | --- |
| 1 m/s，20 s，2 s 加速/制动 | speed RMSE 0.000495 m/s；最终速度 0.000673 m/s；max pitch 0.151°；max abs(xi_delta) 0.153 mm；QP feasible=1；NMPC fault=0 |
| 1 m/s，0.20 rad/s，90° 左转 | yaw 89.292°；半径误差 -0.080%；speed RMSE 0.00836 m/s；max abs(xi_delta) 2.434 mm；NMPC fault=0 |
| 0.1 m/s，0.08 rad/s，360° 左转 | 物理稳定完成；yaw 360.125°；max abs(xi_delta) 1.282 mm；严格 residual gate 未全过 |

CSV 摘要在 evidence/，文件含原始分类列。修改 gate 定义时不要只复制表中一句结论。

## 8. 当前 baseline 的准确能力边界

已验证或基本验证：

- 三维 Simscape plant 与柔性轮地接触能够闭环运行；
- 16-state/12-input NMPC 与 12-DoF weighted WM-WBC 全链路工作；
- 平地 1 m/s 启动、匀速、制动；
- 平地 1 m/s、0.2 rad/s、90° 左转；
- 左右轮位差在已测直线和转向中保持毫米/亚毫米量级；
- WBC QP 和 NMPC timing 在已测平地工况中满足当前门限。

未验证或明确未通过：

- 1 m/s 右转镜像、1 m/s 360°、最终配置下 720°/1800°；
- 当前 paper_eq12 baseline 的正式外扰稳定工作域；
- terrain adaptation。

terrain failure map 显示 flat 的 common wheel-position P-P 约 1.585 mm，但 5° 坡与 20 mm/0.5 m wave 均约 55 mm；10°/15° 坡和 20–80 mm sharp step 会进入严重失败。QP feasible ratio 即使在这些失败 case 中仍可能为 1，因为 interaction-wrench slack 保持优化问题数学可行。

因此本目录应被称为“平地验证的控制基线和 terrain failure baseline”，不能称为“已完成地形适应的机器人”。

## 9. 默认配置不可静默改变

当前冻结行为：

- paper_eq12_v1，dynamicsVersion=7；
- 16-state/12-input NMPC，Ts=0.02 s，N=20；
- weighted WM-WBC，Ts=0.005 s；
- Eq.(12) wheel-relative acceleration feedforward 开启；
- WC-01 fixed；
- WC-02 legacy default，full material-point candidate 默认关闭；
- normal task 为 N0；
- 外层 differential drift stabilizer 和 QP 后髋膝补偿关闭；
- FRF、task-attribution、weight probe、hierarchy PoC 默认关闭；
- 无 terrain-dependent weight schedule；
- oracle terrain map 只允许作为诊断，不能解释为在线 terrain estimator。

若要改变这些行为，先复制候选，更新 model technical contract，重新构建 solver（如状态/OCP 变化），并在 Phase 中完成结构测试和性能回归；不要直接覆盖本 baseline。

## 10. 输出和清理

- work/、slprj/、*.slxc、simulation results 和 raw MAT 是本地生成物；
- 长测试前切换到明确的、被忽略的输出目录，避免 CSV 散落在源码目录；
- evidence/ 只保留小型、人工选择且能说明基线身份的摘要；
- 不要把生成物复制到 docs/models。

## 11. 关键文件地图

| 职责 | 文件 |
| --- | --- |
| 参数、平衡构型和控制默认值 | model/simulate/two_legs/startup.m |
| 三维 plant | model/simulate/two_legs/source.slx |
| 16-state nonlinear dynamics | model/simulate/two_legs/full_base_body_dynamics.m |
| NMPC state-space/CasADi 接口 | model/simulate/two_legs/full_base_wheel_state_space.m |
| NMPC OCP | model/simulate/two_legs/full_base_nmpc_ocp.m |
| NMPC state reconstruction | model/simulate/two_legs/full_base_nmpc_state_signal.m |
| NMPC horizon reference | model/simulate/two_legs/full_base_nmpc_reference.m |
| NMPC output guard | model/simulate/two_legs/full_base_nmpc_command.m |
| 12-DoF WM-WBC | model/simulate/two_legs/spatial_two_leg_qp_core.m |
| 主链装配 | model/simulate/two_legs/configure_symmetric_two_leg_simulink.m |
| NMPC block 装配 | model/simulate/two_legs/configure_base_nmpc_simulink.m |
| 直线 regression | calibration/studies/2026_08_stage1_performance/run_stage1_straight_cases.m |
| 转向 regression | calibration/studies/2026_08_two_leg_model_tests/test_large_yaw_turning_simulink.m |

模型语义、公式和迁移注意事项见 ../../docs/models/simulink_mpc_wm_wbc_baseline.md。
