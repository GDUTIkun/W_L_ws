# Point-subspace equivalence audit

结论：`C-REFERENCE_POINT_MISMATCH`。

本审计只使用 compatible-H0、tick0、frozen Model B、actual bilateral two-point contacts、production
contact frame 与现有 aggregate wrench reference；没有修改 controller、projector、contact、solver、
gain/weight，也没有运行 trajectory。

## Direct result

从 actual 两个 3D point forces 以 production ordering
`f=[Fr1,Fl1,Fn1,Fr2,Fl2,Fn2]` 独立重建
`w=[Fr,Fl,Fn,Mr,Ml,Mn]=G_p f`。左右 `G_p` 均 rank 5，nonzero condition number 分别为
`50.0000038 / 50.0000012`；重建 singular values 与既有 oracle 的最大差为
`2.22e-16`，left-null collinearity 为 `1`。

当前 `P_w=diag(I3,I3-aa^T)` 与 actual orthogonal projector `P_G=G_pG_p^dagger` **不相等**：

| side | `||Pw-Pg||2` | `||Pw-Pg||F` | mutual containment `2`-norm | max principal angle | null collinearity |
| --- | ---: | ---: | ---: | ---: | ---: |
| left | `2.152654918e-4` | `3.044313780e-4` | `2.152654918e-4` | `2.152654935e-4 rad` | `0.999999976830` |
| right | `1.520424464e-4` | `2.150204897e-4` | `1.520424464e-4` | `1.520424469e-4 rad` | `0.999999988442` |

actual missing directions（符号任意）为：

```text
left  [-2.152654918e-4, ~0, ~0, ~0, +0.999999976830, ~0]
right [-1.520424464e-4, ~0, ~0, ~0, +0.999999988442, ~0]
```

current `P_w` 删除的是 pure `Ml=[0,0,0,0,1,0]`。因此“几乎纯 Ml”只能描述近似方向，不能证明
projector exactness；actual missing direction 含有 material `Fr/Ml` coupling。

## Reconstruction and reference-point closure

canonical allowed directions、compatible-H0 wrench 与 current candidate nominal physical solution
均做 minimum-norm point-force reconstruction。最大 residual norm 为 left/right
`2.152654918e-4 / 1.520424464e-4`；candidate nominal physical solution 自身 residual 为
`1.521421424e-5 / 5.564357335e-6`。reverse reconstruction 同样显示 current `P_w` 会删除 actual
realizable component，最大 basis residual 为 `2.152654887e-4 / 1.520424464e-4`。

差异来自 reference convention：production reference 比 actual contact midpoint 沿 normal 偏移
`-2.152654968e-4 / -1.520424481e-4 m`。按标准 wrench transport
`M_new=M_old+(r_old-r_new) x F` 移到 contact midpoint 后，actual left-null direction 与 pure `Ml`
collinearity 为 `1`；transported/direct `G_p` parity `<=3.47e-18`，往返 closure 为 `0`。移到 wheel
center 则 collinearity 约 `0.99876`，进一步确认 pure-`Ml` 不是 intrinsic two-point geometry fact，
而是 reference-point-specific statement。

## Authority consequence

current Ml-deletion candidate 不是 actual two-point force-image projector。因此 `DG46P-EQ FAIL` 只能
解释为 **approximate Ml-deletion candidate fails equilibrium**，不能解释为 exact R1 repair fails
equilibrium。先前 `R2-CONTACT_RESPONSE_MISMATCH_AFTER_R1` 的 authoritative status 被本审计
supersede，降级为 **non-authoritative / candidate-specific**；本轮只记录 attribution，不定义或实现
新的 exact repair。

Machine-readable authority：
[formal-v1](evidence/automated/point-subspace-equivalence-formal-v1/point-subspace-equivalence.json) 与
[fresh replay-v1](evidence/automated/point-subspace-equivalence-replay-v1/summary.json)。
