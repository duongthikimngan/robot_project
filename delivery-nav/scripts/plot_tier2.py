"""Hình tầng 2: tỉ lệ thành công (±Wilson 95%) và sai số định vị, DR vs bốn cấu hình EKF.
    python scripts/plot_tier2.py                 # sai số định vị trên MỌI lần chạy
    python scripts/plot_tier2.py --only-success  # chỉ các lần chạy thành công
"""
import argparse
from pathlib import Path
import numpy as np
import pandas as pd
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

ROOT = Path(__file__).resolve().parents[1]
D = ROOT / "results" / "giaidoan5_experiment"
FILES = {
    "DR":         "exp_corridor_lm8_d1.5_tu8_dronly.csv",
    "EKF lm3":    "exp_corridor_lm3_d1.5_tu8.csv",
    "EKF lm3 r4": "exp_corridor_lm3_r4_d1.5_tu8.csv",
    "EKF lm8":    "exp_corridor_lm8_d1.5_tu8.csv",
    "EKF lm8 r4": "exp_corridor_lm8_r4_d1.5_tu8.csv",
}
COL = dict(noise="noise", tracker="tracker", success="success", loc="loc_err_rms", localizer="loc")
LEVELS = ["low", "mid", "high"]


def wilson(k, n, z=1.96):
    if n == 0:
        return 0.0, 0.0, 0.0
    p = k / n
    den = 1 + z * z / n
    c = (p + z * z / (2 * n)) / den
    h = z * np.sqrt(p * (1 - p) / n + z * z / (4 * n * n)) / den
    return p, c - h, c + h


def load():
    out = {}
    for lab, f in FILES.items():
        df = pd.read_csv(D / f)
        if COL["tracker"] in df.columns:
            df = df[df[COL["tracker"]] == "fixed"]
        df = df[df[COL["localizer"]] == ("dr" if lab == "DR" else "ekf")]
        print(lab, df.groupby(COL["noise"], observed=True).size().to_dict())   # phải là 30 mỗi mức
        out[lab] = df
    first = next(iter(out.values()))
    print("cột:", list(first.columns))
    print("giá trị cột nhiễu:", first[COL["noise"]].unique())
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--only-success", action="store_true")
    a = ap.parse_args()
    dfs = load()
    w = 0.8 / len(dfs)
    off = lambda i: (i - (len(dfs) - 1) / 2) * w

    # --- tỉ lệ thành công ---
    fig, ax = plt.subplots(figsize=(9, 4.5))
    for i, (lab, df) in enumerate(dfs.items()):
        ps, lo, hi = [], [], []
        for lv in LEVELS:
            s = df[df[COL["noise"]] == lv][COL["success"]]
            p, l, h = wilson(int(s.sum()), len(s))
            ps.append(p); lo.append(p - l); hi.append(h - p)
        ax.bar(np.arange(len(LEVELS)) + off(i), ps, w, yerr=[lo, hi], capsize=2, label=lab)
    ax.set_xticks(range(len(LEVELS))); ax.set_xticklabels(LEVELS)
    ax.set_xlabel("mức nhiễu odometry"); ax.set_ylabel("tỉ lệ thành công (±Wilson 95%)")
    ax.set_ylim(0, 1.05); ax.grid(axis="y", alpha=.3); ax.legend(fontsize=8, ncol=3)
    plt.tight_layout(); plt.savefig(ROOT / "results" / "fig_tier2_success.png", dpi=150); plt.close()

    # --- sai số định vị ---
    fig, ax = plt.subplots(figsize=(9, 4.5))
    for i, (lab, df) in enumerate(dfs.items()):
        data = []
        for lv in LEVELS:
            d = df[df[COL["noise"]] == lv]
            if a.only_success:
                d = d[d[COL["success"]] == 1]
            data.append(d[COL["loc"]].dropna().to_numpy())
        bp = ax.boxplot(data, positions=np.arange(len(LEVELS)) + off(i), widths=w * 0.9,
                        patch_artist=True, flierprops=dict(markersize=2))
        for b in bp["boxes"]:
            b.set_facecolor(f"C{i}"); b.set_alpha(.6)
        ax.plot([], [], color=f"C{i}", lw=6, alpha=.6, label=lab)
    ax.set_xticks(range(len(LEVELS))); ax.set_xticklabels(LEVELS)
    ax.set_yscale("log"); ax.set_xlabel("mức nhiễu odometry"); ax.set_ylabel("loc_err_rms (m)")
    ax.grid(axis="y", alpha=.3); ax.legend(fontsize=8, ncol=3)
    tag = "_onlysuccess" if a.only_success else ""
    plt.tight_layout(); plt.savefig(ROOT / "results" / f"fig_tier2_locerr{tag}.png", dpi=150)
    print("đã lưu hình vào", ROOT / "results")


if __name__ == "__main__":
    main()