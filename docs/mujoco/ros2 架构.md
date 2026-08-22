推荐你用这一套，简单、清晰，也方便以后 MuJoCo→真机直接切换：

```
          同一套 Controller Core
      Planner + NMPC + WBC
                  │
              ROS2 Node
                  │
        ┌─────────┴─────────┐
        ↓                   ↓
 MuJoCo Adapter       Hardware Adapter
        ↓                   ↓
     MuJoCo          STM32 + 真机
     
     
     
     
     ROS2 Controller Node
│
├── 订阅 RobotState
│
├── 调用 Controller Core
│      ├── Planner
│      ├── NMPC
│      └── WBC
│
└── 发布 TorqueCommand
```

核心做法只有三点：

- **MuJoCo 和真机都走 ROS2 对外接口。**
- **控制算法本体用ros2实现，但要和robotstate分层。**
- **MuJoCo 和真机都转换成同一个 `RobotState`，都接收同一个 `TorqueCommand`。**

统一的数据结构可以先做得很简单：

```
RobotState
- base pose
- base velocity
- joint position[6]
- joint velocity[6]
- contact state
- timestamp

TorqueCommand
- joint torque[6]
```

MuJoCo：

```
mjData
↓
MuJoCo Adapter
↓
RobotState
↓
ROS2
↓
Controller
↓
TorqueCommand
↓
MuJoCo Adapter
↓
mjData.ctrl
```

真机：

```
Encoder + IMU
↓
Hardware Adapter
↓
RobotState
↓
ROS2
↓
同一个 Controller
↓
TorqueCommand
↓
STM32
↓
电机
```

频率上也保持简单：

```
MuJoCo physics：1000 Hz，内部自己跑
WBC：200 Hz
NMPC：50 Hz
电机力矩环：留在驱动器/STM32
```

不要让 MuJoCo 的每个 physics step 都走 ROS2，也不要让电流环走 ROS2。

最终你只需要做到：

> **MuJoCo 和真机使用同一套 ROS2 状态/力矩接口；切换仿真和实物时，只替换最底层 Adapter，Controller 不改。**

这是我最推荐、也最容易落地的一套。