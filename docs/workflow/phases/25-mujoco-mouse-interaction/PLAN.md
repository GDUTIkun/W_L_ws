# Phase 25: MuJoCo mouse interaction — PLAN

Status: `complete`

## Goal

让 P24 的 current nominal NMPC viewer 支持原生鼠标视角控制和对选中刚体的临时 force/torque 拖动。

## Scope

- 普通左/右/中键拖动与滚轮控制 MuJoCo free camera。
- Ctrl+左键拖动施加转矩，Ctrl+右键拖动施加力；释放鼠标立即停止扰动。
- overlay 明确显示操作说明；仅 viewer 分支调用 `mjvPerturb` 和 `xfrc_applied`。

## Out of Scope

- 不改变 Controller/Adapter/NMPC/WBC、headless log、scene 或真机通信。
- 不做持久外力、参数编辑、录制、手柄或远程 UI。

## Frozen Decisions

- 直接复用 MuJoCo `mjv_moveCamera`、`mjv_select`、`mjvPerturb` 和 `mjv_applyPerturbForce`，不实现自定义鼠标物理。
- 鼠标扰动是本次仿真中的 viewer-only temporary external wrench，不是 Controller input 或 formal case。

## Tasks

| ID | Task | Deliverable | Validation | Status |
| --- | --- | --- | --- | --- |
| P25-T01 | 实现原生 camera/perturb callbacks | mouse callbacks、selection、force/torque application | build + GUI smoke | done |
| P25-T02 | 文档和审查 | entry 操作说明、REVIEW/RECORD | regression + manual entry | done |

## Acceptance Criteria

- [x] Viewer-only interaction does not alter headless behavior.
- [x] Camera and perturb hooks are backed by MuJoCo native APIs.
- [x] Build/test and GUI initialization pass.

## Execution Notes

- 2026-08-29 P25-T01：以 MuJoCo 3.7.0 安装的 `sample/basic.cc` 和 `simulate.cc` 为 API reference，直接接入 `mjv_moveCamera`、`mjv_select`、`mjv_initPerturb`、`mjv_movePerturb` 与 `mjv_applyPerturbForce`。普通 left/right/middle drag 与 scroll 作用于 free camera；Ctrl+left/right select body 后分别 rotate/translate perturb，释放 left/right 清除 active perturb。扰动在物理 substep 的 `mj_step` 前叠加到 viewer-local `xfrc_applied`。
- 2026-08-29 P25-T02：Release build PASS；fresh headless 10-tick NMPC CSV smoke PASS；`DISPLAY=:0 ... --viewer true ... --ticks 5` GLFW/MuJoCo GUI smoke PASS；`colcon test --packages-select wheel_leg_core wheel_leg_ros wheel_leg_mujoco && colcon test-result --verbose` = `26 tests, 0 errors, 0 failures, 0 skipped`。环境无鼠标事件注入工具，最终拖动方向由 documented desktop entry 人工确认；不影响该 viewer-only implementation 的 build/initialization evidence。

## Blockers

None.
