# Phase 43 Review

结论：`REWORK`  
日期：2026-08-30

## Findings

1. **BLOCKING — DG43-EQ / selection unresolved.** A/B/C/D的tick0 native wheel qdd gate全部FAIL；
   B/C/D虽把common `ddxi`从-0.120206压到约-0.0313 m/s²，right native qdd仍约-3.09 rad/s²。
2. **BLOCKING — DG43-CONTACT/RATE/BASE/WBC.** 没有nominal case达到10 s；首失效在tick28–139。
   所有candidate common rim rate峰值0.836–1.146 m/s，远超0.25 m/s；A/B部分失触，C/D主要越过
   base-rotation gate，部分B/D同时越过slack gate。
3. **BLOCKING — DG43-PERT未进入。** 上游mandatory gates失败，四类small perturbation按冻结顺序
   未运行，不能宣称stabilization。
4. **PASS — baseline/provenance/replay.** tick111 right contact loss精确复现；formal-v3与fresh
   replay-v2 summary/snapshot/candidate/gate结果一致，baseline内部双运行semantic error=0。
5. **PASS — implementation isolation.** component test证明B与C task独立、D=B+C；core 17/17、
   adapter 6/6测试PASS；fixed-state whole-vector residual `7.11e-14`、contact reconstruction 0。
6. **PASS — no fake wrench stabilization.** wrench fidelity未相对baseline恶化，但这不覆盖其余FAIL。

## Disposition

Phase43保持`review`，分类P43-U，不创建RECORD、不把ROADMAP标为complete、不进入Phase44 tracking。
后续需要新的技术decision Phase；当前证据只授权调查controller reduced wheel rows与MuJoCo actual
native wheel/contact-constrained dynamics之间的剩余实现差异，不预授权调权重或新增控制结构。

已确认：Phase34 run=false、12D NMPC run=false、16D repair=false、plant modification=false、
contact modification=false。
