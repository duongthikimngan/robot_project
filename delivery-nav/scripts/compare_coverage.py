import pandas as pd
from scipy.stats import binomtest, wilcoxon

R = 1.0


def load(lm):
    d = pd.read_csv(rf"results\giaidoan4_localization\diag_fail_corridor_{lm}_high.csv").set_index("seed")
    mc = [c for c in d.columns if c.startswith("miss_j")]
    return d, (d[mc] > R).sum(axis=1)


a, na = load("lm3_r4")
b, nb = load("lm8_r4")
seeds = a.index.intersection(b.index)
na, nb = na.loc[seeds], nb.loc[seeds]
print(f"{len(seeds)} seed chung; điểm trượt: lm3 {int(na.sum())}, lm8 {int(nb.sum())}")

okA, okB = na == 0, nb == 0
b01 = int((~okA & okB).sum())     # lm3 trượt, lm8 không
b10 = int((okA & ~okB).sum())     # lm3 không, lm8 trượt
print(f"lượt không trượt điểm nào: lm3 {int(okA.sum())}, lm8 {int(okB.sum())}")
print(f"cặp bất đồng: lm3 trượt và lm8 không = {b01}; lm3 không và lm8 trượt = {b10}")
if b01 + b10:
    print("McNemar chính xác: p =", round(binomtest(b01, b01 + b10, 0.5).pvalue, 5))
print("Wilcoxon ghép cặp theo seed (số điểm trượt): p =", round(wilcoxon(na, nb).pvalue, 5))

print("\nlượt mất nhất quán (NEES > 4,5):")
print("  lm3:", sorted(a.index[a.nees_mean > 4.5].tolist()))
print("  lm8:", sorted(b.index[b.nees_mean > 4.5].tolist()))
print("lm8: seed trượt nhiều nhất:", int(nb.idxmax()), "với", int(nb.max()), "điểm")