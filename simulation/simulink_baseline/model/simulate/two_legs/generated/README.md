# Local Acados generated runtime

此目录下除本文件和 `.gitignore` 外的内容是派生、平台相关的 Acados 生成物，不属于 W_L_ws 的权威产品源码，因此不进入 Git。

本次迁入在本机保留了：

- `paper_eq12_v1/`：`source.slx` 实际使用的 Windows x64 full 16-state solver runtime；
- `base_wheel_8state_nmpc/Ts_0p01_N_30_paper_common_v2/`：optional 8-state common-mode generated source bundle，当前没有顶层 S-Function。

权威模型/OCP 源码位于本目录上一级的 `full_base_body_dynamics.m`、`full_base_wheel_state_space.m`、`full_base_nmpc_ocp.m` 与 `build_base_nmpc_solver.m`。在另一主机或 MATLAB release 上，应提供受控 Acados/CasADi 依赖并重新生成；不要把缺失或 ABI 不兼容的 MEX 当成控制模型错误。
