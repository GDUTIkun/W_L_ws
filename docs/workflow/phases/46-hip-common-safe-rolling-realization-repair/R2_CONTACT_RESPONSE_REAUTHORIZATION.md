# R2 Contact-Response Repair Re-Authorization

本轮只做 fixed-state source/oracle/authorization audit，不修改 production controller、QP、MuJoCo
XML、contact 参数或 solver。最终分类为：

```text
E-R2-SOURCE-CLOSED-BUT-LAW-NOT-TRUSTED
```

## Source freeze and current production law

fresh current slip-common contact gap为 `-0.753272490427`，harmful-cross contact contribution为
`+4.676730109204`。production 42D QP变量为 `nudot(12), tau(6), aggregate wrench(12),
slack(12)`，并已 hard-enforce reduced dynamics：

```text
M nudot - B tau - Aw_L Pg_L w_L - Aw_R Pg_R w_R = -h.
```

它还包含 closure reduction、contact Jacobian/bias、37-row/side wrench cone、soft contact
acceleration、xi/rolling 与 interaction-wrench tasks，以及 exact R1 projector。故 current QP 已经
couple `nudot/tau/wrench`；重新写 dynamics 会触发 `REJECT-NO-NEW-PHYSICAL-RELATION`。

第一条缺失关系是：没有 constitutive/complementarity equation 把 QP 选择的 contact wrench绑定到
同一 `tau`、state 和 active contact regime 在 plant 中产生的 reaction。

## Native source oracle

fixed active-set MuJoCo law精确满足：

```text
M qacc = qfrc_smooth + J^T f
f = D (aref - J qacc)
```

Schur 消元为：

```text
qacc = (M + J^T D J)^-1 (qfrc_smooth + J^T D aref).
```

oracle reconstruction errors：qacc `4.34e-14`、row force `3.11e-15`、point force
`1.07e-14`、generalized force `2.84e-14`、observable `6.33e-15`；full dynamics和
constitutive closure分别 `2.84e-14/1.33e-15`。去除已判定 bookkeeping 的两个 weak closure
rows，qacc/observable仅变化 `2.88e-7/5.64e-8`，不重新打开 closure repair。

geometry 为 NONMATERIAL；friction active set稳定，不能认定为 root cause。`D/aref` 是 exact
oracle 的必要部分，但本轮没有隔离出 compliance 的 standalone causal fraction，故不授权独立
compliant family。

## Candidate and stop decision

coupled primal law A 与 Schur law B 是同一个 physical law 的两种形式；A/B equivalence 为 YES。
Stage S same-tau oracle解释 contact gap为 `100%`。但 Stage R 将 Schur relation加入 diagnostic
controller QP 后，没有得到可信 controller counterfactual：

- H0 maximum constraint violation：`0.0368512`；
- H0 predicted `ddxi_c/slip_c`：`-6.86299/-0.783202`；
- branch split：`3.12034`；scale convergence error：`8.29579`；
- R1仍为 machine-scale PASS，但 H0、feasibility、branch 和 scale均 FAIL。

因此 Stage S PASS 不能解释为 repair PASS。A/B 都因 closed-loop diagnostic law integration不可信
而拒绝授权；C未独立归因，D residual feedback继续 deferred。没有 selected/authorized R2 law，
production numerics完全不变。

```text
R2 AUTHORIZED FOR ONE IMPLEMENTATION CANDIDATE: NO
R2 IMPLEMENTED: NO
NEXT ALLOWED ACTION: additional contact-response source attribution only
```

Evidence: [formal-v1](evidence/automated/r2-contact-response-reauthorization-formal-v1/r2-contact-response-reauthorization.json)
and [fresh replay-v1](evidence/automated/r2-contact-response-reauthorization-replay-v1/summary.json).
