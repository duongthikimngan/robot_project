# scripts/plot_pareto.py
import glob, re
import pandas as pd
import matplotlib.pyplot as plt

OUT = r"results\giaidoan5_experiment"
files = sorted(glob.glob(OUT + r"\exp_corridor_lm8_d*.csv"))

rows = []
for f in files:
    d = float(re.search(r"_d([\d.]+)\.csv", f).group(1))
    df = pd.read_csv(f)
    df = df[df["loc"] == "ekf"]
    for (noise, trk), g in df.groupby(["noise", "tracker"]):
        rows.append(dict(d=d, noise=noise, tracker=trk,
                         steer=g["steer_rate"].mean(),
                         miss=g["miss_max"].mean(),
                         rms=g["cte_rms"].mean()))
res = pd.DataFrame(rows)

for ykey in ["miss", "rms"]:
    fig, axes = plt.subplots(1, 3, figsize=(13, 4), sharey=False)
    for ax, noise in zip(axes, ["low", "mid", "high"]):
        s = res[res.noise == noise]
        fx = s[s.tracker == "fixed"].sort_values("d")
        ax.plot(fx["steer"], fx[ykey], "o-", label="d cố định")
        for _, r in fx.iterrows():
            ax.annotate(f"{r['d']:g}", (r["steer"], r[ykey]),
                        textcoords="offset points", xytext=(4, 4), fontsize=8)
        ad = s[s.tracker == "adaptive"]
        ax.plot(ad["steer"], ad[ykey], "r*", ms=14, label="thích nghi")
        ax.set_title(f"nhiễu {noise}")
        ax.set_xlabel("steer_rate (thấp = mượt)")
        ax.set_ylabel(ykey + (" (thấp = tốt)"))
        ax.grid(alpha=.3)
    axes[0].legend()
    fig.tight_layout()
    fig.savefig(OUT + rf"\pareto_{ykey}_ekf.png", dpi=150)