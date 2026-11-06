import argparse
import pandas as pd
from scipy.stats import binomtest, spearmanr

ap = argparse.ArgumentParser()
ap.add_argument("--map", default="corridor")
ap.add_argument("--lm", default="lm3_r4")
ap.add_argument("--noise", default="high")
ap.add_argument("--rreach", type=float, default=1.0)
a = ap.parse_args()

path = rf"results\giaidoan4_localization\diag_fail_{a.map}_{a.lm}_{a.noise}.csv"
d = pd.read_csv(path)
mc = [c for c in d.columns if c.startswith("miss_j")]
if not mc:
    raise SystemExit(f"{path} chưa có cột miss_j*: chạy lại diag_lm3r4.py --part fail "
                     f"--map {a.map} --lm {a.lm} --noise {a.noise}")
miss = d[mc] > a.rreach
d["n_miss"] = miss.sum(axis=1)
print(f"{path}: {len(d)} lượt, tổng {int(d.n_miss.sum())} điểm trượt "
      f"({d.n_miss.mean():.2f}/lượt), {int((d.n_miss == 0).sum())} lượt không trượt điểm nào")
print("tần suất trượt từng điểm (cả nhóm):")
print(miss.mean().round(2).to_string())

for thr in (4.0, 4.5, 6.0):
    bad = d.nees_mean > thr
    print(f"\nngưỡng NEES {thr}: mất nhất quán {int(bad.sum())}, nhất quán {int((~bad).sum())}")
    for name, m in (("mất nhất quán", bad), ("nhất quán", ~bad)):
        if m.sum():
            print(f"  {name}: {int(d.loc[m, 'n_miss'].sum())} điểm trượt, "
                  f"{d.loc[m, 'n_miss'].mean():.2f} điểm/lượt")

rho, p = spearmanr(d.nees_mean, d.n_miss)
print(f"\ntương quan hạng NEES trung bình và số điểm trượt: rho = {rho:.2f}, p = {p:.3f}")

cols = [c for c in ("seed", "n_miss", "nees_mean") if c in d.columns]
print("\ncác lượt trượt từ 3 điểm trở lên:")
print(d.loc[d.n_miss >= 3, cols].round(2).to_string())

ok = d.nees_mean <= 4.5
print(f"\nnhóm nhất quán ({int(ok.sum())} lượt): tần suất trượt, khoảng tin cậy Wilson 95%")
for c in mc:
    k, m = int(miss.loc[ok, c].sum()), int(ok.sum())
    lo, hi = binomtest(k, m).proportion_ci(method="wilson")
    print(f"  {c}: {k}/{m} = {k / m:.2f}   [{lo:.2f}, {hi:.2f}]")