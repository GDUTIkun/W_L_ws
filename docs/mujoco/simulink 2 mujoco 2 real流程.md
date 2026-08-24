# Simulink → MuJoCo → Real 分层验证路线

当前路线的核心目标不变：

> **Simulink 保留为算法对照基线；MuJoCo 先完成多刚体实现正确性验证，再与真机做共同辨识和分层闭环验证。**

Simulink 中已验证的算法使用了简化刚体假设，而 MuJoCo 和真机的目标 plant 是多刚体系统。因此，从 Simulink 迁移到 MuJoCo 不能只是“换一个仿真器调参”，而必须分开回答两个问题：

1. **实现正确性：** 坐标、运动学、多刚体拓扑和动力学实现是否正确；
2. **模型一致性：** MuJoCo 参数、执行器和接触特性是否与真机足够一致。

只有第一个问题通过后，才用 MuJoCo 和真机的共同实验回答第二个问题。MuJoCo 运动学、单腿动力学和实验接口会直接被后续控制与真机验证复用，不是抛弃性工作。

## 四个放行门

| 放行门 | 要回答的问题 | 通过后才进入 |
| --- | --- | --- |
| A. 语义统一 | 同名变量是否代表同一物理量 | MuJoCo 运动学测试 |
| B. MuJoCo 实现正确 | 多刚体几何与动力学是否自洽 | MuJoCo–真机共同辨识 |
| C. Plant 基本一致 | 执行器、重力、摩擦、惯量与耦合是否可信 | 轮子接触与关节闭环 |
| D. 分层闭环通过 | MuJoCo 与真机是否在当前控制层一致 | 更高层控制 |

每完成一层：

```text
MuJoCo PASS
↓
真机对应低风险验证
↓
MuJoCo / Real 基本一致
↓
进入下一层
```

如果：

```text
MuJoCo PASS
Real FAIL
```

则停在当前层排查模型失配，不继续向上增加控制复杂度。

---

# 0. 坐标系 / 单位 / 物理语义统一

首先统一：

```text
world frame
body frame
controller frame

左右腿定义
hip / knee / wheel 正方向
wheel rolling 正方向

torque 正方向
roll / pitch / yaw 正方向

角度 / 角速度单位
力矩单位
减速比定义
```

真机人工缓慢转动关节，确认：

```text
MuJoCo q > 0
=
Real q > 0
=
Controller q > 0
```

同时检查：

```text
encoder q
encoder dq
IMU orientation
IMU angular velocity
```

这一步只解决：

> **三个系统中的同一个变量，是不是代表同一个物理量。**

没通过之前不进入后面的辨识。

---

# 1. MuJoCo 运动学测试

坐标系、单位和物理语义冻结后，立即在 MuJoCo 做运动学测试。这一步验证当前运动学算法与 MuJoCo 几何定义基本正确，不做真机参数辨识，也不用闭环效果替代几何验证。

首先冻结：

- 单腿刚体拓扑与关节顺序；
- 每个 joint、body、site 的 frame；
- joint axis、零位、正方向和限位；
- hip、knee、wheel center、contact point 的定义；
- wheel rolling direction；
- Controller state 到 MuJoCo q、dq 的映射。

在零位、边界附近及若干典型工作姿态比较：

| 对照项 | 基准 |
| --- | --- |
| 解析/现有算法 FK | MuJoCo body/site pose |
| 解析/现有算法 Jacobian | MuJoCo Jacobian |
| Jacobian 速度预测 | 有限差分位移 |
| 关节正向微小扰动 | 末端实际移动方向 |

覆盖至少包括：

- hip position / orientation；
- knee position / orientation；
- wheel center position / orientation；
- contact point；
- wheel rolling direction；
- contact Jacobian。

测试计划必须根据数值量级冻结位置、姿态和 Jacobian 容差；没有超差的典型姿态与方向性错误才可 PASS。

> **运动学 PASS 只证明几何、坐标和 Jacobian 实现基本正确，不证明质量、惯量、执行器或接触模型正确。**

---

# 2. 建立完整的单腿多刚体模型

运动学通过后，先在 MuJoCo 中建立一条可独立验证的完整单腿 plant，再做 MuJoCo–真机共同辨识。

第一版边界为：

- base fixed；
- single leg；
- no ground contact；
- 保留全部单腿刚体和关节；
- 在已约定的关节边界输入 torque；
- 可观测 q、dq、joint torque 和所需 body/site state。

模型必须显式包含：

- 真实刚体拓扑和几何；
- 各刚体 mass、COM、inertia；
- 关节轴、限位和必要阻尼；
- 未被 CAD 刚体惯量覆盖的执行器反射惯量接口；
- 重力与传感器/状态输出语义。

每个质量、质心和惯量参数都要记录来源、单位和是否待辨识。此时允许使用 CAD、称重或规格书给出的 nominal 参数，但不能把 nominal 值写成真机已校准结论。

此阶段不引入轮地接触、floating base、WBC 或 NMPC，避免多个未验证因素同时进场。

---

# 3. MuJoCo 单腿动力学内部验证

接入真机辨识前，对完整单腿多刚体模型做一轮模型内部验证。这里验证的是“结构和实现是否自洽”，不是“参数是否已与真机匹配”。

按照从静态到动态的顺序验证：

1. **静力学：** 多个姿态下的重力方向、重力力矩和静态平衡；
2. **惯量矩阵：** \(M(q)\) 的维度、对称性、正定性与姿态依赖性；
3. **正逆动力学一致性：** 给定 \((q,\dot q,\ddot q)\) 计算力矩，再用相同状态和力矩恢复加速度；
4. **耦合测试：** 单关节激励时其他关节的惯性耦合方向和数量级；
5. **开环回放：** 在无接触、明确初值和受限力矩下记录 \(q\)、\(\dot q\)、\(\ddot q\) 与能量收支。

Simulink 只在与其简化刚体假设重合的工况下作为算法回归对照。两者不一致时，先判定是实现错误还是已知的模型忠实度差异，不为了追求波形一致而把 MuJoCo 退化成简化模型。

通过条件：运动学无超差、正逆动力学在冻结容差内自洽、惯量矩阵无异常，并且已有可重复的单腿开环测试。达到后才进入真机侧低风险实验和共同辨识。

---

# 4. 拆机前基础传感器检查

这里只做最基础的数据可信度检查。

检查：

```text
q
dq
Iq
torque estimate
```

确认：

```text
方向正确
单位正确
零点正常
没有明显跳点
采样频率已知
驱动器内部滤波情况已知
```

## 当前原则

拆机状态下如果 raw data 已经比较干净：

$$
\boxed{\text{不额外做低通滤波}}
$$

不要为了流程完整而强行加入滤波器。

拆机实验主要保存：

```text
timestamp
q_raw
dq_raw
Iq_raw
tau_cmd
tau_est
```

供后面分析。

最终在线 RobotState 使用的正式低通参数，不在这里确定。

---

# 5. 拆机执行器测试

测试对象：

```text
电机
+
减速器
+
驱动器
```

作为一个完整关节执行器模块。

原流程要求在进入动力学前先回答：

> **控制器输出 1 N·m，关节到底真正收到多少 N·m？**

---

## 5.1 静态力矩标定

装置：

```text
固定执行器
↓
输出轴
↓
刚性力臂
↓
Load Cell
```

测：

$$
\tau_{\rm real}=Fr
$$

给不同正负力矩：

```text
-τ
...
0
...
+τ
```

记录：

```text
τcmd
Iq
F
τreal
```

拟合：

$$
\boxed{
\tau_{\rm real}
=
k_\tau\tau_{\rm cmd}+b
}
$$

得到：

```text
torque scale
torque bias
deadzone
正负方向不对称
```

解决：

$$
\boxed{
\tau_{cmd}\rightarrow\tau_{real}
}
$$

---

## 5.2 执行器自身摩擦

关闭主动输出，用力臂缓慢拖动输出轴。

分别测：

### 启动力矩

```text
静止
↓
逐渐增加外力
↓
刚开始转动
```

得到：

$$
\tau_s
$$

即 static friction / breakaway torque。

### 低速 Coulomb friction

输出轴已经缓慢运动后，记录维持运动所需力矩：

$$
\tau_c
$$

正反方向分别得到：

```text
τs+
τs-

τc+
τc-
```

---

## 5.3 执行器等效惯量

[[拆机力矩测试]]

---

# 6. 装回机器人

拆机阶段完成：

```text
torque mapping
执行器自身 friction
Jactuator
```

以后将执行器装回整机。

从这里开始研究：

> **执行器 + 连杆 + 轴承 + 装配 + 线束共同组成的真实机器人。**

---

# 7. 更新 MuJoCo 基础执行器模型

把拆机得到的信息补到 MuJoCo。

主要包括：

```text
torque mapping
未建模 actuator reflected inertia
必要的 actuator friction
```

其中：

$$
J_{\rm actuator}
$$

不能不加区分地整块重复加入。

如果某些：

```text
输出轴
法兰
执行器外壳
```

已经作为 CAD 刚体惯量进入 MuJoCo，就不能再次重复算进关节附加惯量。

MuJoCo joint 的附加惯量主要用于补：

```text
CAD 没有建模的
电机转子反射惯量
减速器内部旋转惯量
```

---

# 8. 装机后的正式 Sensor / RobotState 测试


[[传感器过低通]]

---

# 9. ROS2 / MuJoCo / Real 统一架构

[[ros2 架构]]

---

# MuJoCo–真机共同辨识约定

从这里开始，目标由“证明 MuJoCo 实现自洽”切换为“辨识并验证 MuJoCo 与真机的差异”。后续正式辨识尽量复用同一套：

- 初始姿态与关节锁定条件；
- 输入力矩或运动轨迹；
- 安全限幅与停止条件；
- RobotState / TorqueCommand 字段和时间语义；
- 采样、日志和数据质量检查；
- 分析脚本、拟合方法、指标与图表。

同一实验先在 MuJoCo 跑通，再以低风险配置在真机执行。脚本和接口的复用是正式工程资产；但 MuJoCo 与真机的参数集、噪声、延迟和安全边界必须分别保存，不能为了共用代码强行设为相同。

上述约定只适用于专门设计、用于形成辨识结论或放行证据的正式实验。临时画图、小测试和调试探针可以轻量处理，不必走完整实验流程；但它们不能直接替代正式 PASS 证据，除非补齐可重复条件、输入、输出和判据。

---

# 10. 静态重力 / Mass / COM 验证

固定 base，使腿悬空。

选择多个：

$$
q_1,q_2,q_3,\ldots
$$

静止：

$$
\dot q=0,\qquad\ddot q=0
$$

因此：

$$
\tau\approx G(q)
$$

真机测：

```text
actual current
actual torque
```

比较：

$$
\boxed{
\tau_{\rm real}
\quad vs\quad
G_{\rm model}(q)
}
$$

如果不同，优先检查：

```text
torque scale
mass
COM
geometry
```

这一阶段主要校准：

```text
mass
COM
gravity model
```

---

# 11. 装机后的关节总摩擦

这时测的不再只是执行器内部摩擦，而是：

$$
\boxed{
\tau_{f,\rm total}
}
$$

它包含：

```text
执行器自身摩擦
+
轴承摩擦
+
装配预紧
+
不同轴
+
机械干涉
+
线束拖拽
+
其他机构摩擦
```

---

## 11.1 选定姿态 $q_0$

固定 base，锁住其他关节。

只让待测关节运动。

---

## 11.2 静止保持

在：

$$
q=q_0
$$

记录：

$$
\boxed{
\tau_{\rm hold}\approx G(q_0)
}
$$

---

## 11.3 正向低速经过 $q_0$

令：

$$
\dot q=+v
$$

缓慢经过同一个 $q_0$，记录：

$$
\tau_+
$$

得到：

$$
\boxed{
\tau_{f,+}
\approx
\tau_+-\tau_{\rm hold}
}
$$

---

## 11.4 反向低速经过 $q_0$

令：

$$
\dot q=-v
$$

再次经过同一个 $q_0$，记录：

$$
\tau_-
$$

得到：

$$
\boxed{
\tau_{f,-}
\approx
\tau_{\rm hold}-\tau_-
}
$$

第一版：

$$
\boxed{
\tau_c
\approx
\frac{
|\tau_{f,+}|+|\tau_{f,-}|
}{2}
}
$$

必要时在 MuJoCo 中加入：

```text
frictionloss
damping
```

原流程也要求在惯量辨识前先弄清摩擦。

---

# 12. 关节等效惯量 $J_{\rm eq}$

前置条件：

```text
torque mapping
↓
gravity
↓
friction
```

基本明确以后，再做惯量辨识。

这一顺序与原流程要求一致。

[[等效转动惯量测量]]

---

# 13. $J_{\rm eq,real}$ ↔ MuJoCo $M_{jj}$

MuJoCo 设置：

```text
base fixed
其他关节 locked
same q0
no contact
```

完整 MuJoCo 模型仍然使用：

```text
各刚体 mass
各刚体 COM
各刚体 inertia
+
未被 CAD 覆盖的 actuator inertia
```

得到：

$$
M(q)
$$

读取对应关节：

$$
M_{jj}(q_0)
$$

最终比较：

$$
\boxed{
J_{\rm eq,real}(q_0)
\quad vs\quad
M_{jj,\rm MuJoCo}(q_0)
}
$$

也可以让 MuJoCo 做完全相同的：

```text
Δτ → ddq
```

实验，得到：

$$
J_{\rm eq,MJ,identified}
$$

理想关系：

$$
\boxed{
J_{\rm eq,real}
\approx
J_{\rm eq,MJ,identified}
\approx
M_{jj,\rm MuJoCo}
}
$$

注意：

> **$J_{\rm eq}$ 是验证完整动力学模型的量，不是 WBC 中直接填入的固定惯量参数。**

---

# 14. MuJoCo–真机完整单腿动力学验证

第 3 节已经验证 MuJoCo 单腿动力学实现自洽；这里不再重新建模，而是使用相同输入、状态定义和分析方法检查 nominal/calibrated MuJoCo 与真机是否一致：

$$
M(q)\ddot q
+
C(q,\dot q)
+
G(q)
+
\tau_f
\approx
\tau
$$

重点验证：

```text
inertia
dynamic coupling
cross-joint response
residual structure
```

而不是只看单个：

$$
M_{jj}
$$

如果残差具有明显的姿态、速度或加速度依赖性，应分别回到 mass/COM、friction、inertia/耦合项检查，不能只靠提高 PD 增益掩盖。

---

# 15. Wheel Contact

单独验证：

```text
normal
rolling
lateral
friction
```

目标：

```text
接触方向一致
力的数量级合理
滚动行为一致
滑移趋势一致
```

不要求每个接触瞬态完全一致。

---

# 16. Joint PD + Gravity Compensation

完成 plant 校准后第一次真正闭环：

$$
\tau
=
K_p(q_d-q)
-K_d\dot q
+G(q)
$$

顺序：

```text
单关节
↓
整条腿
```

验证：

```text
定点保持
小角度阶跃
正反方向
扰动恢复
torque limit
velocity limit
```

如果：

```text
MuJoCo PASS
Real FAIL
```

仍然优先检查：

```text
state
torque mapping
friction
filter / delay
physical parameters
```

不进入 WBC。

---

# 17. Floating-base 简单站立

关节级闭环可靠后释放 base。

第一版只考虑：

```text
z
pitch
leg posture
wheel position
```

真机：

```text
吊架 / 保护架
低 torque limit
低 velocity limit
E-stop
```

验证：

```text
floating-base dynamics
two-leg coupling
wheel-ground contact
```

---

# 18. Weighted WBC

逐层恢复：

```text
动力学硬约束
↓
torque limits
↓
friction limits
↓
interaction wrench
↓
soft contact
↓
common leg / wheel task
↓
differential suppression
↓
regularization
```

始终：

```text
MuJoCo PASS
↓
Real PASS
↓
继续
```

---

# 19. NMPC

先人工给：

```text
wrench_ref
```

证明：

```text
RobotState
↓
WBC
↓
tau
↓
MuJoCo / Real
```

稳定。

再接：

```text
reference
↓
NMPC
↓
wrench_ref
↓
WBC
↓
tau
```

---

# 20. Roll / Yaw / Turning

逐步增加：

```text
roll recovery
↓
yaw recovery
↓
small yaw-rate
↓
turning
↓
continuous turning
↓
large yaw
```

真机从低速度、低 yaw-rate 开始。

---

# 21. Differential Identification

最后用同样的辨识流程分别研究：

```text
MuJoCo
vs
Real
```

比较：

```text
输入实现比例
频率响应
主导带宽
free-run
yaw-rate 工作点依赖
```

最终允许：

```text
Model_M
≠
Model_R
```

只要能够得到合理的 nominal model + robust margin。

---

# 最终主链路

```text
0. 冻结坐标 / 单位 / 符号 / 状态语义
↓
1. MuJoCo FK / Jacobian / frame / rolling direction 测试
↓
2. 建立 fixed-base、no-contact 的完整单腿多刚体模型
↓
3. MuJoCo 静力学、M(q)、正逆动力学、耦合与开环回放自检
   ── Gate B：先证明模型实现正确，再进入真机辨识
↓
4. 拆机前基础传感器检查
   raw 数据干净即可，不正式定低通
↓
5. 拆机 torque mapping / 执行器摩擦 / Jactuator
↓
6. 装回机器人
↓
7. 更新 MuJoCo actuator / armature
↓
8. 装机 Sensor / RobotState 正式测试与最终滤波
↓
9. ROS2 / MuJoCo / Real 接口统一
↓
10. MuJoCo–真机共同 gravity / mass / COM 辨识
↓
11. 装机关节总摩擦
↓
12. Jeq_real 辨识
↓
13. Jeq_real ↔ Mjj_MuJoCo
↓
14. MuJoCo–真机完整单腿 inertia / dynamic coupling 验证
   ── Gate C：plant 基本一致
↓
15. wheel contact
↓
16. Joint PD + gravity compensation
↓
17. floating-base 简单站立
↓
18. Weighted WBC
↓
19. NMPC
↓
20. roll / yaw / turning
↓
21. differential identification
```

现在这条链可以概括为：

$$
\boxed{
\text{定义正确}
\rightarrow
\text{运动学正确}
\rightarrow
\text{多刚体动力学实现正确}
\rightarrow
\text{力矩与状态可信}
\rightarrow
\text{MuJoCo / Real 参数一致}
\rightarrow
\text{接触可信}
\rightarrow
\text{闭环控制}
}
$$

其中最关键的顺序约束是：

> **坐标与物理语义统一后，先完成 MuJoCo 运动学测试和完整单腿多刚体动力学自检；只有 MuJoCo 实现正确性通过，才开始 MuJoCo–真机共同辨识。**

同时保留原有滤波原则：拆机阶段 raw 数据足够干净就不加低通；最终滤波参数等执行器装回整机后，在真实结构振动和工作状态下再正式确定。
