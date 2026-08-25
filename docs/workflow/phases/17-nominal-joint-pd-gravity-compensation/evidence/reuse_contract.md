# Phase 17 复用与非覆盖契约

- Controller 算法读取普通 C++ `ControllerConfig`；默认 `zero`，显式 `joint_pd_gravity` profile 才启用 reference、gains、gravity 和 limits。
- current nominal profile 以 canonical offset、三项解析谐波、gains 和 simulation-only limits 固化；模型 revision 改变时创建新 profile/config，不继承本次系数或 PASS。
- Phase 16 C++ runner 仍是唯一 physics/control loop；Phase 17 Python 只编排 case、计算 oracle/指标、比较 replay 和写 manifest。
- 旧 CSV 基础列保持名称与语义，新 diagnostics 只追加。Phase 16 zero/fault wrapper 已在扩展 runner 上回归 PASS。
- wrapper 拒绝非空输出目录；`formal`、补强 C++ profile 交叉检查后的 `formal-v2` 和补强配置注入后的 `formal-v3` 都保留，最终审查以 `formal-v3` 为准。
- SolidWorks revision、identified profile 或未来真机复现必须使用新 run ID 和新 manifest；本次 nominal evidence、PLAN/REVIEW/RECORD 不原地覆盖。
