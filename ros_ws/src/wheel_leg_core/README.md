# wheel_leg_core

ROS、MuJoCo 和硬件传输无关的 C++17 公共类型、契约校验与 Controller Core 安全骨架。当前 Core 不含控制算法；对有效状态仅产生六路零力矩。

独立验证：

```bash
cmake -S ros_ws/src/wheel_leg_core -B build/wheel_leg_core -DBUILD_TESTING=ON
cmake --build build/wheel_leg_core
ctest --test-dir build/wheel_leg_core --output-on-failure
```
