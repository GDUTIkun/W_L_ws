# MuJoCo → Real 当前更新路线

当前路线的核心目标不变：

> **不要直接把 Simulink 控制器和参数整包搬到 MuJoCo / 真机，而是从传感器、执行器、机械模型、动力学到控制器逐层验证。**

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

# 1. 拆机前基础传感器检查

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

# 2. 拆机执行器测试

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

## 2.1 静态力矩标定

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

## 2.2 执行器自身摩擦

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

## 2.3 执行器等效惯量

[[拆机力矩测试]]

---

# 3. 装回机器人

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

# 4. 更新 MuJoCo 基础执行器模型

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

# 5. 装机后的正式 Sensor / RobotState 测试


[[传感器过低通]]

---

# 6. ROS2 / MuJoCo / Real 统一架构

[[ros2 架构]]

---

# 7. 运动学验证

暂时不进入完整动力学。

先验证：

```text
FK
Jacobian
hip position
knee position
wheel center
contact point
wheel rolling direction
contact Jacobian
```

选择多个典型姿态：

```text
q0
q0 + Δq1
q0 + Δq2
...
```

比较：

```text
解析模型
vs
MuJoCo
```

确保：

```text
position
orientation
Jacobian
```

一致。

---

# 8. 静态重力 / Mass / COM 验证

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

# 9. 装机后的关节总摩擦

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

## 9.1 选定姿态 $q_0$

固定 base，锁住其他关节。

只让待测关节运动。

---

## 9.2 静止保持

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

## 9.3 正向低速经过 $q_0$

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

## 9.4 反向低速经过 $q_0$

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

# 10. 关节等效惯量 $J_{\rm eq}$

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

# 11. $J_{\rm eq,real}$ ↔ MuJoCo $M_{jj}$

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

# 12. 完整动力学验证

进一步检查：

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
```

而不是只看单个：

$$
M_{jj}
$$

---

# 13. Wheel Contact

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

# 14. Joint PD + Gravity Compensation

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

# 15. Floating-base 简单站立

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

# 16. Weighted WBC

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

# 17. NMPC

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

# 18. Roll / Yaw / Turning

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

# 19. Differential Identification

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
0. 坐标 / 单位 / 符号
↓
1. 拆机前基础传感器检查
   raw 数据干净即可，不正式定低通
↓
2. 拆机静态 torque mapping
↓
3. 拆机执行器自身摩擦
↓
4. 拆机 Jactuator
↓
5. 装回机器人
↓
6. 更新 MuJoCo actuator / armature
↓
7. 装机 Sensor / RobotState 正式测试
↓
8. FFT / PSD → 确定最终 fc / τ
↓
9. ROS2 / RobotState 统一
↓
10. FK / Jacobian
↓
11. gravity / mass / COM
↓
12. 装机关节总摩擦
↓
13. Jeq_real ↔ Mjj_MuJoCo
↓
14. 完整 inertia / dynamic coupling
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
\text{力矩可信}
\rightarrow
\text{执行器可信}
\rightarrow
\text{状态可信}
\rightarrow
\text{机械模型可信}
\rightarrow
\text{动力学可信}
\rightarrow
\text{接触可信}
\rightarrow
\text{闭环控制}
}
$$

其中最新调整最关键的一点是：

> **拆机阶段 raw 数据足够干净就不加低通；最终滤波参数等执行器装回整机后，在真实结构振动和工作状态下再正式确定。**
