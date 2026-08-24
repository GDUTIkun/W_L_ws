# Simulink baseline snapshot manifest

## Source identity

- Source workspace: D:\Workspace\CodeWorkspace
- Source snapshot: model\simulate\proformance_test
- Source Git commit: acb36ec229354142f4c77bb57073e8f590c418fb
- Source commit time: 2026-08-24T09:56:01+08:00
- Source scoped worktree state: clean
- Target: D:\Workspace\W_L_ws\simulation\simulink_baseline
- Import date: 2026-08-24
- Verified release inherited from source snapshot: MATLAB R2024b

## Import boundary

Imported:

- the two model/code runtime dependencies called by spatial_two_leg_qp_core;
- all authored MATLAB and SLX assets under model/simulate/two_legs;
- current 16-state paper_eq12_v1 solver runtime and the optional 8-state common-mode generated-source bundle;
- Stage-1 performance and large-yaw/turning regression sources;
- MATLAB Project metadata;
- five small evidence summary CSV files, including the target-path import smoke summary.

Excluded:

- calibration/results, raw MAT and full time series;
- figures and historical investigation reports;
- work, slprj, SLXC and autosave files;
- CMake build/object/cache directories;
- all generated solver variants not selected by current startup.m;
- external Acados/CasADi repositories.

Generated runtime binary files are Windows-specific and do not establish cross-platform portability. 整个 generated runtime 在目标仓库中作为本机 replay asset 被忽略，不作为产品源码提交；新 clone 必须从受控 artifact 恢复或重建。MATLAB validation 还会创建 ignored work/cache files，所以 raw physical file count 不用作 identity check。

## Key SHA-256 comparison

| Relative file | SHA-256 / relation | Import result |
| --- | --- | --- |
| model/simulate/two_legs/source.slx | source 64A87B64E090419809DEAA0B3656106D255F62DC074AAC7F360304382BE34845; target EF6C7876B156DC36A1AEC0EBCA169D9205B3A042408760D585A88D3860530A58 | copied, then target diagnostic port contract refreshed from 143 to 198 and saved |
| model/simulate/two_legs/source_common.slx | FCAF0F952068BE8A0ADA8C3E13F84351E990110C260427B25BD3AE0E3ADB7D23 | yes |
| model/simulate/two_legs/startup.m | 11A18BF6A00EE2B9543F300CDE3F6B26F1930E681687A81F9497EF07D7E27E2D | yes |
| model/simulate/two_legs/spatial_two_leg_qp_core.m | 9B0DBAC5E12CD51FD0FF40F3BCE28E609B09992ADC46459A6F25736343939C00 | yes |
| model/simulate/two_legs/full_base_body_dynamics.m | B428A70518871B849790A584FAA43A32D818E3480B1C779E417AE6B45BCB1575 | yes |
| model/simulate/two_legs/full_base_nmpc_ocp.m | 7AA79D01E5E83A3774080192C4A15CBC0C23F98080CC2AA4C1F57D0B6EF704B5 | yes |
| model/code/differential_leg_force_stabilizer.m | D75C41CEC3691FF39CBA56FD5F2A72F02F95D4609C40DE78A7EA9E051B48C78C | yes |
| open_proformance_test.m | 76216FC30B8E17A56F5C40217031288D7B03F27C4E26EC8DEA6E7823D376F308 | yes |
| run_performance_smoke.m | 8562CF60708701EBD68E007F3DF907FF7B715087598DC6CF0894BFEE748F429C | yes |

## Runtime solver selection

startup.m selects:

1. optional generated source: model/simulate/two_legs/generated/base_wheel_8state_nmpc/Ts_0p01_N_30_paper_common_v2
2. active full runtime: model/simulate/two_legs/generated/paper_eq12_v1

The second bundle provides acados_solver_sfunction_full_base_wheel_16state_nmpc. The first bundle does not currently provide a top-level 8-state solver S-Function and is not required by source.slx. CMake object/cache trees were excluded; generated C/H, build scripts, JSON, MAT build signature, Simulink block and full-runtime MEX/DLL files required for the frozen Windows replay were retained.

## Target-only interface refresh

The source snapshot combined the current 198-value coupled-QP diagnostic code with an older source.slx block declaration of 143 values. This prevented simulation before any motion command. In the target copy, the project-owned configure_symmetric_two_leg_simulink(true) updater changed only the append-only diagnostic port/demux contract to width 198 (version 08-04-PAIR-HQP) and saved source.slx. Control parameters, controller source, plant and solver were unchanged. The refreshed target model passed update, contract tests and the 5 s smoke.

The target was subsequently reduced to a runnable baseline boundary. Offline
symbolic/planar derivation scripts and the historical drift-helper unit test were
removed from model/code; the two stabilizer functions remain because the active
spatial_two_leg_qp_core calls them even though their baseline gains are disabled.

Any change to dynamicsVersion, OCP dimensions, constraints, state/input order or build signature invalidates binary reuse.
