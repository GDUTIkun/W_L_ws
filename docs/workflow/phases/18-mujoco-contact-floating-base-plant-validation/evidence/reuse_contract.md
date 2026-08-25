# Phase 18 复用与非覆盖契约

- model revision、probe/full scene、contact profile、case matrix、thresholds、MuJoCo 版本和 runner 均由 config 与 SHA-256 manifest 绑定。
- nominal、未来 SolidWorks revision 和 identified profile 使用同一 runner/schema，但必须使用新 config 和新输出目录。
- runner 在读取/仿真前拒绝非空输出目录；旧正式 evidence 不原地覆盖。
- `2026-08-25-formal` 到 `formal-v4` 保留逐步增加 friction-power、双向 lateral/base-twist 和 config-driven radius 的运行；默认 authority 路径与最终源码绑定后，正式 authority 是追加的 `2026-08-25-formal-v5`，前序运行均未删除或改写。
- Phase 14/15/16/17 回归写入 `data/experiments/2026-08-25-phase18-regression-*/raw`，不覆盖其历史 evidence。
- 新 revision 必须重新验证 collision mask、wheel mesh/radius/axis、normal/rolling/lateral、floating touchdown 和历史回归；不得继承 nominal PASS。
