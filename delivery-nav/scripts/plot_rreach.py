"""Tỉ lệ thành công theo ngưỡng giao hàng R_REACH, tính lại từ miss_max (không chạy lại thí nghiệm)."""
from pathlib import Path
import numpy as np
import pandas as pd
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

ROOT = Path(__file__).resolve().parents[1]
D = ROOT / "results" / "giaidoan5_experiment"
FILES = {"DR": "exp_corridor_lm8_d1.5_tu8_dronly.csv",
         "EKF lm8": "exp_corridor_lm8_d1.5_tu8.csv",
         "EKF lm3 r4": "exp_corridor_lm3_r4_d1.5_tu8.csv"}
COL = dict(noise="noise", tracker="tracker", success="success", miss="miss_max", coll="collisions")
LEVELS = ["low", "mid", "high"]
R = np.arange(0.5, 1.501, 0.05)
R_REACH = 1.0

fig, axs = plt.subplots(1, 3, figsize=(13, 4), sharey=True)
for lab, f in FILES.items():
    df = pd.read_csv(D / f)
    if COL["tracker"] in df.columns:
        df = df[df[COL["tracker"]] == "fixed"]
        df = df[df["loc"] == ("dr" if lab == "DR" else "ekf")]
    ok = ((df[COL["miss"]] < R_REACH) & (df[COL["coll"]] == 0)).astype(int)
    print(f"{lab}: lệch so với cột success đã lưu = {int((ok != df[COL['success']]).sum())} dòng")
    for ax, lv in zip(axs, LEVELS):
        d = df[df[COL["noise"]] == lv]
        s = [((d[COL["miss"]] < r) & (d[COL["coll"]] == 0)).mean() for r in R]
        ax.plot(R, s, label=lab)
for ax, lv in zip(axs, LEVELS):
    ax.axvline(R_REACH, color="k", ls=":", lw=1)
    ax.set_title(f"nhiễu {lv}"); ax.set_xlabel("R_REACH (m)"); ax.grid(alpha=.3)
axs[0].set_ylabel("tỉ lệ thành công"); axs[0].legend()
plt.tight_layout(); plt.savefig(ROOT / "results" / "fig_success_vs_rreach.png", dpi=150)