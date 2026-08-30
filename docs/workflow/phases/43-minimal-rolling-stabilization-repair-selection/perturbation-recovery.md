# Phase 43 Perturbation Recovery

P43-1/2冻结为 direction-normalized rim-rate common/differential `0.02 m/s`初态扰动；P43-3/4
冻结为左右轮体2 N、50 ms水平外力的common/differential pulse，并从pulse撤除后评价。该定义避免
通过改absolute wheel angle伪造`xi`位移，也不修改plant/contact参数。

正式结果：`NOT ENTERED`。

所有候选均先在DG43-EQ失败，且随后nominal rollout无一通过10 s mandatory gates。按冻结顺序，
runner没有生成perturbation rollout，`summary.perturbations={}`。这不是DG43-PERT PASS；它表示上游
gate阻止进入本层。不得用未运行的扰动测试支持稳定性结论。
