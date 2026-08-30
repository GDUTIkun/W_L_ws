# Phase 43 Nominal Regulation

尽管所有候选在DG43-EQ失败，formal仍运行冻结的10 s nominal层以保留诊断证据；这些运行不能
越过首个 independent failure，也不用于绕过tick0 gate。

| Candidate | last tick | first independent failure | peak common rim rate m/s | xi common error m |
| --- | ---: | --- | ---: | ---: |
| A | 28 | left contact loss | 1.14572 | 0.05373 |
| B 2.5 Hz | 116 | normalized slack 0.05081 | 0.88966 | 0.01953 |
| B 3.5 Hz | 120 | base rotation 0.37845 rad（同时slack 0.05585） | 0.94409 | 0.01846 |
| B 5 Hz | 125 | left contact loss | 0.88000 | 0.01144 |
| C 2.5/3.5/5 Hz | 107/107/106 | base rotation >0.35 rad | 0.99260/0.93334/0.88149 | 0.01559/0.00989/0.00544 |
| D 2.5/3.5/5 Hz | 120/125/139 | slack或base rotation > frozen gate | 0.86206/0.86116/0.83625 | 0.01928/0.01105/0.00580 |

没有候选达到10 s。所有候选的native common rim rate都超过0.25 m/s gate；因此C虽改善xi error，
仍没有关闭hidden native-rate mode。WBC hard/torque多数组合保持可行，但这不能补偿rate/base/contact
FAIL。程序均在首个 independent failure row立即停止。
