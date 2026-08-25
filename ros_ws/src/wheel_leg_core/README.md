# wheel_leg_core

ROS、MuJoCo 和硬件传输无关的 C++17 公共类型、契约校验与 Controller Core。默认配置对有效状态保持六路零力矩；显式合法的 `joint_pd_gravity` 配置可启用 canonical Joint PD、versioned 解析重力补偿、求和后限幅和逐项 diagnostics。

独立验证：

```bash
cmake -S ros_ws/src/wheel_leg_core -B build/wheel_leg_core -DBUILD_TESTING=ON
cmake --build build/wheel_leg_core
ctest --test-dir build/wheel_leg_core --output-on-failure
```
