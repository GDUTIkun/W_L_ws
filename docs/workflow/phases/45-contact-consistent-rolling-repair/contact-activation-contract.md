# Contact activation contract

- valid = wheel-floor contact exists、normal load finite、tangent norm >1e-9、row/bias/slip finite；
- tick0 valid且`Fn>=5 N`同步 enable；以后 inactive 需连续2 ticks `Fn>=5 N`才 re-enable；
- active时`2 N < Fn < 5 N`保持；contact invalid或`Fn<=2 N`立即 disable；
- normal load只决定有效性，不进入tracking objective；所有 transition和左右状态必须记录。
