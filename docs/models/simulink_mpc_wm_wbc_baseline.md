# 轮腿机器人 Simulink MPC–WM-WBC baseline 模型说明

Status: frozen behavioral baseline  
Snapshot date: 2026-08-24  
Authoritative assets: ../../simulation/simulink_baseline/  
Primary model: ../../simulation/simulink_baseline/model/simulate/two_legs/source.slx

## 1. 文档目的和事实优先级

本文让没有旧对话、旧 workspace 或 Research Vault 记忆的新 agent 能够理解当前模型，并把相同物理语义迁移到 MuJoCo、C++ Controller Core 和真机接口。

事实优先级：

1. baseline 内当前 MATLAB/Simulink 源码与 startup.m；
2. SNAPSHOT_MANIFEST.md 和 evidence 下的摘要；
3. 本文；
4. 旧研究笔记。

若本文与代码冲突，以代码为准，并同步更新本文和对应 Phase。不要依据旧聊天记录静默改变模型。

## 2. 一句话定位

这是一个完整三维双轮腿闭环：

- Simscape Multibody + compliant wheel-ground contact 是高保真 plant；
- 上层 16-state、12-input NMPC 用论文 Eq.(12) 的 wheel-relative-to-base dynamics 预测 base 与左右 wheel position；
- 下层 12-DoF weighted whole-body QP 把左右 wheel–leg interaction wrench 请求转成六个关节力矩；
- WBC interaction-wrench slack 允许下层在全身动力学、接触、执行器和 soft tasks 冲突时偏离 MPC 请求；
- 当前行为在平地已通过关键直线和转向测试，但 terrain adaptation 没有通过。

    trajectory / velocity / height / yaw reference
      -> online common wheel-position planner
      -> 16-state / 12-input NMPC
      -> interaction-wrench command guard
      -> 12-DoF weighted WM-WBC
      -> six joint torques
      -> Simscape Multibody + compliant contact
      -> measured states and feedback

## 3. 当前版本锚点

| Item | Frozen value |
| --- | --- |
| wheel-relative dynamics | paper_eq12 |
| dynamicsVersion | 7 |
| full NMPC build tag | paper_eq12_v1 |
| NMPC states / inputs | 16 / 12 |
| NMPC sample time | 0.02 s |
| NMPC horizon | N=20, 0.40 s |
| WBC generalized DoF | 12 |
| WBC decision size | 36 |
| WBC sample time | 0.005 s |
| outer QP-post anti-split | disabled |
| paper acceleration feedforward to WBC | enabled |
| WC-01 | fixed |
| WC-02 full material-point candidate | default off |
| normal diagnostic modes | N0 baseline; Noff/Ncomp off |
| FRF/weight probe/task attribution/hierarchy POC | default off |

default-off 的研究仪表可以保留，但不能被描述成生产控制环。

## 4. 坐标、方向和单位

### 4.1 控制器坐标

- X：机身前向；
- Y：机身侧向；
- Z：竖直向上；
- 姿态顺序：roll、pitch、yaw；
- position 和 linear velocity 在 world frame N 表达；
- NMPC interaction wrench 在 body-aligned controller frame 表达；
- 单位为 m、s、rad、N、N·m、kg。

Simscape 物理轴排列与控制器排列不同。WBC 中存在有意的坐标置换，例如 physical state 的 [1,3,2] 重排。迁移时必须按语义重建映射，不能按数组下标直觉抄写。

### 4.2 yaw

- positive yaw / yaw-rate 对应当前 world reference 的左转；
- 左转 case 使用正 yaw，右转使用负 yaw；
- yaw measurement 连续展开，不在正负 pi 处 wrap；
- 360°/720° reference、error 和 derivative 依赖 continuous heading。

MuJoCo/C++ 若使用 quaternion，接口层仍需生成连续 heading。

### 4.3 wheel position

左右 wheel position 不是 wheel spin angle，而是 wheel center 相对 base 的前向位置：

$$
\xi_i=e_x^T r_{iw}^{B}, \qquad i\in\{L,R\}.
$$

$$
\xi_c=\frac{\xi_L+\xi_R}{2}, \qquad
\dot\xi_c=\frac{\dot\xi_L+\dot\xi_R}{2}.
$$

本文规范差模：

$$
\xi_\Delta=\frac{\xi_R-\xi_L}{2}, \qquad
\dot\xi_\Delta=\frac{\dot\xi_R-\dot\xi_L}{2}.
$$

spatial_two_leg_qp_core.m 的历史内部变量为：

$$
\xi_{\mathrm{diff,legacy}}
=\frac{\xi_L-\xi_R}{2}
=-\xi_\Delta.
$$

移植差模反馈、日志或门限时必须处理此负号。

## 5. Simscape plant

机器人包含一个 6-DoF floating base，左右各一条 thigh–shank–wheel leg。每条腿的 hip、knee、wheel spin 三个 revolute joints 均接收 torque command。每个轮子使用一个 Spatial Contact Force。

### 5.1 几何和惯性

| Parameter | Value | Status |
| --- | ---: | --- |
| thigh length L1 | 0.35 m | simulation assumption |
| shank length L2 | 0.35 m | simulation assumption |
| thigh mass m1 | 1.20 kg each | simulation assumption |
| shank mass m2 | 0.80 kg each | simulation assumption |
| link section | 0.04 x 0.04 m | simulation assumption |
| wheel radius rho | 0.08 m | simulation assumption |
| wheel mass mw | 0.35 kg each | simulation assumption |
| wheel axial inertia Iw | 0.00112 kg·m² | 0.5 mw rho² |
| base body mass | 3.0 kg | simulation assumption |
| base size | 0.45 x 0.45 x 0.32 m | simulation assumption |
| hip half spacing d | 0.20 m | simulation assumption |
| plant total mass | 7.70 kg | model sum |
| NMPC non-wheel mass mb | 7.00 kg | base + four links |

7.70 kg 包含两轮；7.00 kg 不包含两轮。Eq.(12) 单独使用 wheel mass/inertia，不能把轮质量再次并入 mb。

上体等效惯量：

$$
I_B=\operatorname{diag}(0.264225,\ 0.076225,\ 0.28925)
\ \mathrm{kg\,m^2},
$$

顺序为 roll、pitch、yaw。

### 5.2 初始平衡

startup.m 数值求解静态平衡：

| Quantity | Value |
| --- | ---: |
| single-leg q0 | [-0.7023717590, 1.1553364373, 0] rad |
| single-leg dq0 | [0, 0, 0] rad/s |
| equilibrium xi | -0.0729388591 m |
| nominal height magnitude | 0.581863003 m |
| one-side vertical wrench reference | 34.335 N |

Simscape 的某些 z 量为负，controller height 使用正几何幅值；不要按日志表面符号翻转 height feedback。

### 5.3 wheel-ground contact

| Item | Value |
| --- | ---: |
| normal stiffness | 20000 N/m per wheel |
| normal damping | 150 N·s/m per wheel |
| transition width | 0.001 m |
| static / dynamic friction | 0.50 / 0.30 |
| critical velocity | 0.02 m/s |
| friction law | Smooth Stick-Slip |

plant 为 compliant contact；WBC 接触任务为理想 kinematic consistency 的 soft task。两者不等价。normal mismatch 是次级因素，但不是已测 terrain failure 主因。

### 5.4 numerical solver

| Item | Value |
| --- | --- |
| solver | variable-step ode15s |
| max step | 0.005 s |
| relative / absolute tolerance | 1e-3 / 1e-4 |

Simscape wall-clock time 不能代替 controller deadline。实时性看 nmpcCpuTime、QP solve time 和 deadline miss。

## 6. 状态重构和参考

full_base_nmpc_state_signal.m 的关键契约：

- hip relative angle 加 base pitch 后再做 absolute leg kinematics；
- xi 来自 wheel center 对 base 的 geometry，不使用 wheel spin；
- yaw continuous unwrap；
- physical angular velocity 转为 local Euler rate；
- 输出为 [time; x16; height]，共 18 个量。

wheel_position_lqr_reference.m 生成 common wheel-position reference：

| Item | Value |
| --- | ---: |
| design frequency / damping | 2 Hz / 1 |
| max xi speed / acceleration | 0.15 m/s / 0.50 m/s² |
| LQR Q / R | [4;1] / 200 |
| xi range | [-0.3584423, 0.3584423] m |
| height schedule | [0.4318630, 0.7318630] m |

左右 NMPC xi reference 相同。转向由 differential Fx distribution 产生 yaw，Eq.(12) 自然预测 differential wheel-position response。

turning_world_reference.m 使用：

$$
v_L=v-d\dot\psi,\qquad
v_R=v+d\dot\psi.
$$

该几何 wheel speed 用于 trajectory/diagnostic，不会把左右 xi reference 改成相反目标。

## 7. 16-state、12-input NMPC

### 7.1 state order

$$
x=
\begin{bmatrix}
p_B^N\\
\phi\\\theta\\\psi\\
v_B^N\\
\dot\phi\\\dot\theta\\\dot\psi\\
\xi_L\\\xi_R\\
\dot\xi_L\\\dot\xi_R
\end{bmatrix}
\in\mathbb{R}^{16}.
$$

| Indices | Meaning | Frame |
| ---: | --- | --- |
| 1:3 | base position | world |
| 4:6 | roll, pitch, yaw | Euler |
| 7:9 | base linear velocity | world |
| 10:12 | Euler rates | local parameter rates |
| 13:14 | xi_L, xi_R | body-forward relative geometry |
| 15:16 | dxi_L, dxi_R | corresponding rates |

### 7.2 input order

$$
u=
\begin{bmatrix}
F_L\\T_L\\F_R\\T_R
\end{bmatrix}
\in\mathbb{R}^{12},
$$

单侧顺序：

$$
u_i=[F_{ix},F_{iy},F_{iz},T_{ix},T_{iy},T_{iz}]^T.
$$

单侧边界：

$$
u_{i,\min}=[-40,0,0,0,-20,0]^T,
\qquad
u_{i,\max}=[40,0,70,0,20,0]^T.
$$

当前只有 Fx、Fz、Ty 被优化；Fy、Tx、Tz 固定为零。yaw 主要来自左右 Fx 差，roll 来自左右 Fz 差，pitch 来自 Fx/Fz moment arm 与 Ty。

### 7.3 base dynamics

$$
m_b\ddot p_B^N=R_B^N(F_L+F_R)+m_bg^N.
$$

$$
I_B\dot\omega_B=M_B-\omega_B\times(I_B\omega_B).
$$

实现保留三维 moment map、gyroscopic term 和 Euler kinematics，但上层仍不优化单独 hip/knee dynamics。

### 7.4 Eq.(12) wheel-relative dynamics

单轮平动/转动加 pure rolling 消元后：

$$
D_w=m_w\rho+\frac{I_w}{\rho}=0.042,
\qquad
a_{Bx}=\frac{F_{Lx}+F_{Rx}}{m_b}.
$$

$$
\ddot\xi_L=-a_{Bx}-\frac{\rho F_{Lx}+T_{Ly}}{D_w},
\qquad
\ddot\xi_R=-a_{Bx}-\frac{\rho F_{Rx}+T_{Ry}}{D_w}.
$$

$$
\ddot\xi_c
=-\frac{F_{Lx}+F_{Rx}}{m_b}
-\frac{\rho(F_{Lx}+F_{Rx})+T_{Ly}+T_{Ry}}{2D_w}.
$$

$$
\ddot\xi_\Delta
=\frac{\rho(F_{Lx}-F_{Rx})+T_{Ly}-T_{Ry}}{2D_w}.
$$

common base acceleration 自动从 differential mode 抵消。该模型已替换旧经验式：

$$
\ddot\xi_\Delta=0.02(F_R-F_L).
$$

不要重新引入 differentialRollingGain。

### 7.5 OCP and solver

- Ts=20 ms，N=20，horizon=0.4 s；
- stage output y=[x;u;u_previous]；
- full reference width 40N+16=816；
- wheel-position rate bound 2 m/s；
- drive coefficient 0.45/sqrt(2)；
- Acados: PARTIAL_CONDENSING_HPIPM, GAUSS_NEWTON, ERK, SQP_RTI。

state weight diagonal：

    [25, 10, 80, 200, 120, 250, 80, 20,
     16, 20, 10, 80, 5, 5, 0.5, 0.5]

one-side input weight：

    [0.04, 0.04, 0.02, 1, 0.04, 1]

one-side increment weight：

    [0.40, 0.40, 0.20, 1, 0.40, 1]

full_base_nmpc_command.m 只接受 status=0、finite wrench、CPU time<=20 ms 的输出，否则 hold previous valid command；无历史时使用 equilibrium wrench，并设置 nmpcFault。

## 8. 12-DoF weighted WM-WBC

### 8.1 input and decision

spatial_two_leg_qp_core.m 接收 46 维：

$$
[\text{NMPC state signal}_{18};
q_L,\dot q_L;
q_R,\dot q_R;
w_{MPC,12};
\text{wheel reference}_4].
$$

$$
q=[p_B;\phi,\theta,\psi;q_L^3;q_R^3]\in\mathbb{R}^{12}.
$$

$$
z=[\ddot q_{12};\tau_6;\lambda_6;s_w{}_{12}]
\in\mathbb{R}^{36}.
$$

- ddq：floating-base and joint acceleration；
- tau：six joint torques；
- lambda：three contact-force coordinates per wheel；
- s_w：12-dimensional interaction-wrench slack。

### 8.2 hard constraints

$$
M(q)\ddot q+h(q,\dot q)=S\tau+J_c^T\lambda.
$$

$$
D_w^{qp}\ddot q+D_\lambda\lambda-s_w
=w_{MPC}-w_{offset}.
$$

| Hard boundary | Value |
| --- | --- |
| per-leg torque | [160,160,45] N·m |
| normal contact force | nonnegative |
| WBC friction coefficient | 0.45 |
| friction | pyramid |
| knee protection | minimum 10° |
| rigid-body dynamics | hard equality |
| contact acceleration | soft task |

interaction-wrench map 显式包含 wheel mass/inertia、gravity/bias、angular momentum、contact force、rho times rolling force torque 和 frame rotation，不是简单 endpoint Jacobian transpose。

### 8.3 soft tasks

| Task | Baseline setting |
| --- | --- |
| wrench slack | normalized penalty 1e5 |
| contact acceleration | priority [25000,200,500], scale [5,2,5] |
| xi common/differential | weight [5,500], scale [5,5] |
| common rolling speed | Kp=40, accel limit 2 m/s², weight 1000 |
| common force feedback | enabled; override disabled |
| base height | Kp=20, Kd=8, accel limit 2 m/s², weight 1000 |
| base pitch | Kp=40, Kd=8, accel limit 6 rad/s², weight 1000 |
| ddq/tau/lambda | small regularization |

contact effective normalized rolling/lateral/normal weights 约为 [1000,50,20]。

### 8.4 Eq.(12) feedforward

WBC 用相同 upper dynamics 与 requested wrench 计算：

$$
\ddot\xi_c^{ff}
=\frac{\ddot\xi_L^{NMPC}+\ddot\xi_R^{NMPC}}{2},
$$

$$
\ddot\xi_{\mathrm{diff,legacy}}^{ff}
=\frac{\ddot\xi_L^{NMPC}-\ddot\xi_R^{NMPC}}{2}.
$$

$$
\ddot\xi_c^*
=\ddot\xi_c^{ff}
-K_{pc}(\xi_c-\xi_c^{ref})
-K_{dc}(\dot\xi_c-\dot\xi_c^{ref}).
$$

differential target 同样为 feedforward 加 PD。反馈带宽 0.5 Hz、阻尼比 1：

$$
K_p\approx9.8696,\qquad K_d\approx6.2832.
$$

该 feedforward 防止 WBC 把 relative-wheel acceleration 拉回零，而 NMPC 同时预测非零值。

### 8.5 solve path

1. 无活跃不等式且 KKT 有效时用 equality solution；
2. 否则用 quadprog interior-point-convex；
3. 使用 previous solution warm start；
4. 失败时回退 equality solution 并做必要限幅。

诊断向量宽度 198，contract version 08-04-PAIR-HQP。indices 1:85 为 legacy diagnostics，86:198 为 append-only differential projection、task attribution 和 pairwise hierarchy diagnostics；这些新增研究通道默认不改变 baseline 控制行为。

## 9. Slack 契约

$$
\boxed{W_c^*=W_c^{mpc}+\varsigma}.
$$

- $W_c^{mpc}$：NMPC requested left/right wheel–leg interaction wrench；
- $\varsigma$：WBC signed wrench slack；
- $W_c^*$：WBC 在 hard constraints 与 weighted soft-task compromise 下得到的 feasible wrench。

$$
\varsigma=W_c^*-W_c^{mpc}.
$$

必须区分：

- QP feasible 只表示数学优化有解；
- small slack 表示上层 wrench 被接近实现；
- large slack 表示 WBC 通过放松上层 wrench tracking 保持可解；
- large slack 不自动等于 actuator saturation；
- slack 可来自 hard constraint 或 soft-task competition；
- $||W^{mpc}-W^*||$ 与 $||\varsigma||$ 是同一信息，不能在指标中重复计数。

terrain failure 中 QP feasible ratio 可保持 1，而 common-Fx slack 与 motion error 明显增大，这正是后续 feasibility-aware 研究的入口。

## 10. contact fixes 和诊断边界

terrainContactMap 可旋转 rolling/lateral/normal basis，并影响 Jc、lambda basis、friction pyramid 与 interaction-wrench/contact map。它不得重新定义 xi。

WC-01 已修：wheel-position task Jacobian 现在对 state reconstruction 使用的同一 xi 公式求导。flat 和 0.10 rad oracle basis 的 left/right/common/differential finite-difference error 约 1e-10。该修复只把旧 slope P-P 从 29.2526 mm 改到 29.1689 mm，所以是真 bug，但不是主因。

WC-02 full material-point candidate 使用：

$$
J_C=J_v+r[n]_\times J_\omega.
$$

它把已测 slope lateral residual 降低约 26.9%，但 xi_c P-P 只改善约 0.026%，所以 baseline 仍使用 legacy，candidate default off。

Noff 在诊断 slope 上只改善约 10.9%，Ncomp 反而恶化；normal task 是次级耦合因素。

## 11. multi-rate execution 与 signals

    sensors
      -> full NMPC state at 5 ms
      -> NMPC state/reference sample at 20 ms
      -> Acados NMPC
      -> command guard
      -> coupled QP input
      -> 5 ms ZOH
      -> coupled WM-WBC
      -> 3+3 joint torque
      -> Simscape plant

| Signal | Meaning |
| --- | --- |
| baseNmpcState | NMPC state |
| nmpcBodyWrench | raw solver wrench |
| nmpcStatus / nmpcCpuTime | Acados result and solve time |
| nmpcFault | command guard fault |
| totalUpperCommand | guarded upper wrench |
| coupledQpSignal | 198-value WBC diagnostic |
| commonWheelStateSignal | common wheel state |
| commonWheelReference | online reference |
| fullBaseNmpcStateSignal | time + x16 + height |
| symmetryLegState | left/right leg state |

## 12. 已验证平地行为

### 12.1 1 m/s start–cruise–brake

| Metric | Result |
| --- | ---: |
| speed RMSE / final speed | 0.000495 / 0.000673 m/s |
| max roll / pitch | 0.05735° / 0.15110° |
| max absolute xi_delta | 0.153 mm |
| QP feasible | 1 |
| NMPC status max / fault | 0 / 0 |
| NMPC P99 / QP max | 7.636 / 2.321 ms |

### 12.2 1 m/s, 0.20 rad/s, 90° left

| Metric | Result |
| --- | ---: |
| yaw / yaw RMSE | 89.292° / 0.947° |
| radius / error | 4.996 m / -0.080% |
| speed RMSE | 0.00836 m/s |
| max absolute xi_delta | 2.434 mm |
| max slack norm | 0.03417 |
| NMPC fault | 0 |
| NMPC P99 / QP max | 9.087 / 2.965 ms |

低速 360° 是 physically stable completion，但 strict wrench/contact residual gate 未全过，不能写成完整 PASS。

## 13. terrain failure boundary

当前 baseline 不具备 terrain adaptation。

| Case | xi_c P-P | common-Fx slack RMS | speed RMSE | max pitch | QP feasible |
| --- | ---: | ---: | ---: | ---: | ---: |
| flat 0.10 m/s | 1.585 mm | 0.0286 | 0.000042 m/s | 0.007° | 1.0 |
| slope 5° | 55.177 mm | 2.132 | 0.001730 m/s | 0.179° | 1.0 |
| slope 10° | 1202.682 mm | 18.937 | 0.317 m/s | 16.712° | 1.0 |
| slope 15° | 1164.676 mm | 27.556 | 0.838 m/s | 9.862° | 1.0 |
| step 20/40/80 mm | 1080–1329 mm | 36.6–134.8 | 0.847–1.130 m/s | 37.4–84.1° | 1.0 |
| wave 20 mm / 0.5 m | 55.634 mm | 3.467 | 0.009787 m/s | 0.227° | 1.0 |

wave case 没有垂直撞击、姿态和 solver status 健康，仍出现同类 slack/xi 放大。因此 mismatch 不是 slope-only 或 contact-transition-only。

当前受证据支持的解释：

$$
\text{terrain working point}
\rightarrow
\text{WBC task compatibility / realization map change}
\rightarrow
\varsigma_{F_x}\text{ magnitude and phase change}
\rightarrow
\text{closed-loop margin loss}.
$$

这不是完整解析稳定性证明，但足以否定“只是一处 contact Jacobian bug”的说法。

在测试范围内，以下均已被排除为唯一主因：contact frame、WC-01、WC-02、normal task、QP infeasibility、单一 scalar weight、测试的 exact hard hierarchy、Eq.(12) coordinate mismatch。

## 14. 迁移必须保持的 semantic contract

1. controller-frame axis；
2. 16-state order 与 continuous yaw；
3. single-side wrench order [Fx,Fy,Fz,Tx,Ty,Tz]；
4. left block before right block；
5. xi 是 wheel-center relative base-forward geometry，不是 spin angle；
6. canonical xi_delta sign；
7. wheel mass/inertia 不重复计入 base mass；
8. NMPC 20 ms / WBC 5 ms multi-rate semantics；
9. W* = W_mpc + slack 的符号；
10. hold-last-valid command guard。

当前质量、惯量、contact stiffness/damping 和 friction 是 simulation assumptions，尚不是 hardware-identified truth。后续真实辨识必须明确替换或限定它们。

推荐一致性验证顺序：

1. kinematics / coordinates；
2. state reconstruction / reference；
3. Eq.(12) one-step dynamics；
4. WBC hard dynamics residual；
5. wrench/slack mapping；
6. joint torque；
7. flat straight；
8. flat turning；
9. terrain failure reproduction。

## 15. 新 agent 接手检查表

开始前：

- 读 simulation/simulink_baseline/README.md；
- 运行 open_proformance_test(false)；
- 检查 which spatial_two_leg_qp_core 和 differential_leg_force_stabilizer；
- 确认 buildTag=paper_eq12_v1、dynamicsVersion=7、fullBaseNmpc.available=true；
- 区分 existing evidence 与 W_L_ws 中实际重跑结果。

修改方程前：

- 明确改的是 plant、NMPC、WBC、reference 还是 diagnostic；
- 写清 state/input/frame/sign impact；
- OCP dynamics/dimensions/signature 变化时重建 solver；
- 先跑 contract tests，再跑 Simscape；
- 建 candidate，不直接覆盖 baseline。

解释结果时：

- 分开 physical stability 与 strict residual gate；
- 分开 QP feasibility 与 wrench fidelity；
- 报告 slack component、sign、scale 和 window；
- 不把 oracle terrain basis 称为 online estimator；
- 不因 case 已写入 runner 就声称通过；
- 不用 Simscape wall time 代替 controller deadline。

## 16. authoritative file map

| Responsibility | Relative path under simulation/simulink_baseline |
| --- | --- |
| entry / path isolation | open_proformance_test.m |
| parameters / defaults | model/simulate/two_legs/startup.m |
| 3D plant | model/simulate/two_legs/source.slx |
| reduced model | model/simulate/two_legs/source_common.slx |
| nonlinear dynamics | model/simulate/two_legs/full_base_body_dynamics.m |
| NMPC state-space | model/simulate/two_legs/full_base_wheel_state_space.m |
| NMPC OCP | model/simulate/two_legs/full_base_nmpc_ocp.m |
| state reconstruction | model/simulate/two_legs/full_base_nmpc_state_signal.m |
| reference / command guard | full_base_nmpc_reference.m / full_base_nmpc_command.m |
| WM-WBC | model/simulate/two_legs/spatial_two_leg_qp_core.m |
| chain configuration | configure_symmetric_two_leg_simulink.m |
| solver build | build_base_nmpc_solver.m |
| straight regression | calibration/studies/2026_08_stage1_performance/ |
| turning regression | calibration/studies/2026_08_two_leg_model_tests/ |
| small evidence | evidence/ |

## 17. 冻结结论

$$
\boxed{
\text{paper_eq12_v1 weighted MPC–WM-WBC, flat-validated,
terrain-failure-characterized, not terrain-adapted}
}
$$

它既是 Simulink-to-C++/MuJoCo 迁移的数值/行为参考，也是后续 feasibility-aware MPC-WBC 研究的 failure baseline；未来算法尚未设计，不能把 slack 已经可观测写成 slack-aware 方法已完成。
