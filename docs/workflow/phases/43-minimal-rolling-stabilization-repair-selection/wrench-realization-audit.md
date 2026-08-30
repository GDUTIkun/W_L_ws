# Phase 43 Wrench Realization Audit

baseline在其tick111失效前的peak `|W_realized-W_requested|` 为3.87368；冻结门允许相对该authority
最多增加0.1。各候选在各自首失效前peak为1.12532–2.75600，故`wrench_realization` gate均PASS。

候选并不是通过严重牺牲requested wrench fidelity获得假稳定：

- A peak 2.75600，maximum normalized slack 0.02756；
- B peak 1.25891–1.66956，slack 0.04872–0.05585；
- C peak 1.12532–1.24692，slack 0.03444–0.04665；
- D peak 1.16594–1.45851，slack 0.04813–0.05484。

B 2.5/3.5 Hz与D 2.5/3.5 Hz仍因slack越过0.05而在DG43-WBC失败。wrench gate PASS不能覆盖
native-rate、base或contact FAIL。machine-readable peak/RMS、per-component requested/realized/slack/
residual与torque margins保存在每个nominal CSV及`summary.json`。
