# Phase 15 Automated Validation — 2026-08-25

## Environment

- Workspace：`/home/t/W_L_ws`
- Python：`./.venv/bin/python`
- MuJoCo：`3.7.0`
- Hardware data/operation：none
- Formal output：`evidence/automated/2026-08-25-nominal/`

## Commands and results

### Coordinate contract

```bash
./.venv/bin/python tools/maintenance/test_mujoco_coordinate_contract.py
```

Result：PASS。FLU、base control frame、joint sign、quaternion continuity 保持通过；real IMU installation 仍是外部门槛。

### Phase 14 regression without evidence overwrite

```bash
./.venv/bin/python tools/experiments/run_mujoco_internal_dynamics.py \
  --output-dir data/experiments/2026-08-25-phase15-phase14-regression/raw
```

Result：PASS。fixture、kinematics、gravity、mass matrix、forward/inverse、constraints、coupling、energy、replay 全部通过。Phase 14 历史 evidence 未修改。

### Phase 15 formal run

```bash
./.venv/bin/python tools/experiments/run_mujoco_closed_chain_kinematics.py
```

Result：PASS。geometry profile、assembly branch、workspace、FK/full Jacobian、reduced Jacobian、velocity/virtual work、left/right symmetry、determinism 全部通过。

### Non-overwrite gate

对同一正式命令再次执行。

Result：exit 1，`Refusing to overwrite non-empty output directory`。正式 evidence 内容未改变。

## Key worst-case metrics

| Metric | Result | Frozen gate |
| --- | ---: | ---: |
| profile↔MuJoCo position | `5.5511e-17 m` | `1e-12 m` |
| mesh↔nominal radius difference | `1.2075e-4 m` | `2e-4 m` |
| closure residual | `3.2474e-15 m` | `1e-11 m` |
| passive reference error | `1.1224e-13 rad` | `1e-10 rad` |
| reverse branch error | `1.1202e-13 rad` | `1e-10 rad` |
| passive block min singular value | `7.3709e-3` | `>=5e-3` |
| passive block condition number | `30.1993` | `<=40` |
| FK position | `2.7756e-16 m` | `1e-10 m` |
| full Jacobian | `6.0490e-16` | `1e-9` |
| `Jc·S` | `1.3341e-16` | `1e-10` |
| reduced analytic↔MuJoCo | `6.0490e-16` | `1e-9` |
| formal FD linear (`1e-5`) | `1.4622e-11` | `5e-7` |
| formal FD angular (`1e-5`) | `1.7825e-11` | `5e-7` |
| velocity | `9.9274e-17` | `5e-7` |
| virtual work | `2.2204e-16 N·m` | `1e-10 N·m` |
| power | `2.7756e-17 W` | `1e-10 W` |
| mirrored position | `2.4980e-16 m` | `1e-10 m` |
| full rerun determinism | `0.0` | `0.0` |

## Reproducibility hashes

- config：`48563f5e8be64ec48b2de7e9efe4c0c02c78b73af0ee6b47f3d98a8e21c9e2dc`
- scene：`c353f3a6bd457a5732fa2464ec66e7faadf9694e31f5fb58be204d63f5686654`
- included model：`a4b1f2e243b6715038b24637325db89cbf820f0ea4eccaafbf832dcaeb71893a`
- runner：`e4b401d1b216c1c24946d6f73c2c61cd7eddc0c7f9c32d206d0dfacd981219e5`

Formal JSON 中 `overall_pass=true`、workspace 共 210 行、`hardware_data_used=false`，且 manifest runner hash 与当前正式 runner 一致。

