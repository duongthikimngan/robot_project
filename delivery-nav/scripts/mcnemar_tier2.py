import pandas as pd
from scipy.stats import binomtest, wilcoxon

D = r"results\giaidoan5_experiment"
PAIRS = [("lm8_r4", "exp_corridor_lm8_r4_d1.5_tu8.csv"),
         ("lm8",    "exp_corridor_lm8_d1.5_tu8.csv"),
         ("lm3_r4", "exp_corridor_lm3_r4_d1.5_tu8.csv"),
         ("lm3",    "exp_corridor_lm3_d1.5_tu8.csv")]
dr = pd.read_csv(rf"{D}\exp_corridor_lm8_d1.5_tu8_dronly.csv")
dr = dr[dr["loc"] == "dr"]

for name, f in PAIRS:
    e = pd.read_csv(rf"{D}\{f}")
    e = e[e["loc"] == "ekf"]
    m = e.merge(dr, on=["noise", "seed", "tracker"], suffixes=("_ekf", "_dr"))
    assert len(m) == len(e) == len(dr), (name, len(m), len(e), len(dr))
    assert not m.duplicated(["noise", "seed", "tracker"]).any()
    print(f"== {name}: {len(m)} cap (ky vong 180 = 3 noise x 2 tracker x 30 seed)")
    for (tr, nz), g in m.groupby(["tracker", "noise"]):
        assert len(g) == 30, (name, tr, nz, len(g))
        b = int(((g.success_ekf == 1) & (g.success_dr == 0)).sum())
        c = int(((g.success_ekf == 0) & (g.success_dr == 1)).sum())
        p = binomtest(b, b + c, 0.5).pvalue if b + c else 1.0
        w = wilcoxon(g.loc_err_rms_ekf, g.loc_err_rms_dr).pvalue
        print(f"  {tr:8s} {nz:5s} succ EKF {g.success_ekf.mean():.2f} DR {g.success_dr.mean():.2f}"
              f" | b={b} c={c} McNemar p={p:.4f}"
              f" | loc_err_rms EKF {g.loc_err_rms_ekf.median():.2f} DR {g.loc_err_rms_dr.median():.2f} Wilcoxon p={w:.2g}")