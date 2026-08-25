# Phase 18 Grounding

- MuJoCo/Python version：`3.7.0`；physics timestep `0.002 s`。
- 原 `wheel_leg.xml` 让全部 73 个 robot mesh geom 参与碰撞；`scence.xml` reset 时存在 11 个内部 CAD mesh contact。
- Phase 18 mask 后只有 `floor=(contype=1,conaffinity=0)`、左右 wheel collision `(0,1)` 为 active geoms；compiled reset contact 为 0，允许 pair 只有两个 wheel-floor pair。
- Phase 15 contract 保持：轮半径 `0.05 m`、轮轴 world `+Y`、正 canonical wheel speed 的无滑轮心方向为 `+X`。
- Adapter 继续只按命名 wheel-floor pair输出二值 contact；公共 `RobotState` 未加入 force/slip 字段。
- Phase 16/17 C++ deterministic loop 未修改。Phase 18 的零控制 plant validation 直接使用同版本 MuJoCo Python binding，以 2 ms 逐步记录接触瞬态。
- Graphify 仅查询已有本地图，没有运行 extract/update。live code 依据 CBM generation `2026-08-25T06:16:31Z` 与直接源码读取。
