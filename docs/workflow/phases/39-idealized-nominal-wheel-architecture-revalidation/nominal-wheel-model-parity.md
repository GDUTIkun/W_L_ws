# Model A/B compiled parity

DG39-00：`PASS`。

Model B 由 Model A 的 fresh-compiled wheel inertial descriptor 显式序列化而来。源码差异仅为
model/include 名称以及两轮 `<inertial>`；compiled 差异仅为两轮 `body_ipos` 的 body-X/Y：

| wheel | Model A radial COM | Model B radial COM |
| --- | ---: | ---: |
| left | `0.12114 mm` | `0` |
| right | `0.11975 mm` | `0` |

`nq/nv/nu/nbody/njnt/ngeom/nsite/neq` 和 body/joint/geom/site/actuator/equality names 全部一致。
非允许 `body_ipos` 差异为 `0`；mass、axial COM、principal inertia、inertial quaternion、joint、
geom、contact、actuator、equality 及已冻结 option 的最大误差均为 `0`，通过 `1e-12` gate。

正式 authority 是 `architecture-revalidation-formal-v2/model_parity.json`；顶层 manifest 同时包含
compiled diff descriptor 与全部 Model A/B、config、runner、executable 和 source authority hashes。
Model B 只获批为 ideal nominal control-validation plant，不是 CAD、真机或 production truth。
