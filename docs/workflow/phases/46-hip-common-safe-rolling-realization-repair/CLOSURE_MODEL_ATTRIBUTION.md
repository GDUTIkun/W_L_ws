# MuJoCo-only closure-model attribution

本轮只审计 compatible-H0/tick0 的 closure operator，不修改 controller、QP、XML、contact、
`solref/solimp` 或 solver。结论为：此前称为 MuJoCo-only 的两个方向不是 exact closure manifold
上的独立 physical modes；它们是有限 closure residual 被当作 exact hard-rank 后产生的 bookkeeping
directions。

## Geometry and rank

`left_leg_closure/right_leg_closure` 各有 `(x,y,z)` 三个 connect rows。四个 material in-plane
rows 的 norm 约为 `0.176--0.194`；两个 `y` row norm 仅为
`1.82591e-4/1.80966e-4`。native singular spectrum 为：

```text
[2.5478487e-1, 2.5478319e-1, 6.1072450e-2, 6.1072347e-2,
 2.5707589e-4, 8.8223053e-8]
```

两侧 site residual 分别为 `[-8.13135e-5, 0, -1.63486e-4] m` 与
`[-8.07009e-5, -2.78e-17, -1.61975e-4] m`。两个 measured `y` rows 与基座转动对该
`x/z` residual 的叉乘 Jacobian 在允许 row sign 后最大误差 `2.74e-17`；joint-relative 成分仅为
roundoff。因此 residual 归零时两个 rows 归零，exact planar closure rank 是 4，不是 6。

## Why the old observable was material

旧 `MJ6-common4` oracle 对全部非零 rows 构造 rigid conditioned inverse。只要 row 未跨越 SVD
cutoff，将两个弱 rows 同时缩放到 `0.5/0.25/0.1`，hard operator 与 scale-1 的最大谱差仅
`7.93e-6`；但把 rows 精确置零会发生 finite qacc jump。旧 slip-common contribution
`-0.044281228354` 的 stored operator closure 为 `1.02e-14`，所以该数字本身复现正确；错误在于
把这个尺度归一化、零极限不连续的 hard-rank counterfactual解释为独立 physical mechanism。

弱 rows 的 `efc_pos <=2.78e-17`、`efc_aref <=6.94e-14`；它们没有独立 stabilization target。
因此该机制分类为：

```text
BOOKKEEPING-HARD-RANK-ARTIFACT-NOT-INDEPENDENT
MJ-ONLY CLOSURE PHYSICAL MECHANISM: NONMATERIAL / NOT INDEPENDENT
CONTACT RESPONSE: UNIQUE MATERIAL REMAINING MISMATCH
R2 CANDIDATE FOR NEXT RE-AUTHORIZATION: YES
R2 AUTHORIZED: NO
```

R2 仍须在下一轮独立冻结 repair law 和 gate 后才可实施。本轮严格停止，不做 contact tuning、
inverse map、precompensation 或 trajectory validation。

Evidence: [formal-v1](evidence/automated/closure-model-attribution-formal-v1/closure-model-attribution.json)
and [fresh replay-v1](evidence/automated/closure-model-attribution-replay-v1/summary.json).
