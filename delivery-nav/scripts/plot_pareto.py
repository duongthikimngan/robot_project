import glob, re
import pandas as pd
import matplotlib.pyplot as plt

OUT = r"results\giaidoan5_experiment"
rows = []

def add(df, trk, d, label):
    df = df[(df["loc"] == "ekf") & (df["tracker"] == trk)]
    for noise, g in df.groupby("noise", observed=True):
        rows.append(dict(label=label, kind=trk, d=d, noise=str(noise),
                         steer=g["steer_rate"].mean(), miss=g["miss_max"].mean(),
                         rms=g["cte_rms"].mean()))

for f in sorted(glob.glob(OUT + r"\exp_corridor_lm8_d*.csv")):
    m = re.search(r"_d([\d.]+)\.csv$", f)
    mt = re.search(r"_d1\.5_tu([\d.]+)\.csv$", f)
    df = pd.read_csv(f)
    if m:
        add(df, "fixed", float(m.group(1)), "fixed")
        if m.group(1) == "1.5":
            add(df, "adaptive", None, "thích nghi tau_up=1")
    elif mt:
        add(df, "adaptive", None, f"thích nghi tau_up={mt.group(1)}")
res = pd.DataFrame(rows)

for ykey in ["miss", "rms"]:
    fig, axes = plt.subplots(1, 3, figsize=(14, 4))
    for ax, noise in zip(axes, ["low", "mid", "high"]):
        s = res[res.noise == noise]
        fx = s[s.kind == "fixed"].sort_values("d")
        ax.plot(fx["steer"], fx[ykey], "o-", label="d cố định")
        for _, r in fx.iterrows():
            ax.annotate(f"{r['d']:g}", (r["steer"], r[ykey]), textcoords="offset points",
                        xytext=(4, 4), fontsize=8)
        for (lab, g), mk in zip(s[s.kind == "adaptive"].groupby("label"), ["^", "s", "*"]):
            ax.plot(g["steer"], g[ykey], mk, ms=11, label=lab)
        ax.set_title(f"nhiễu {noise}"); ax.set_xlabel("steer_rate (thấp = mượt)")
        ax.set_ylabel(ykey + " (thấp = tốt)"); ax.grid(alpha=.3)
    axes[0].legend(fontsize=8); fig.tight_layout()
    fig.savefig(OUT + rf"\pareto_{ykey}_ekf_v2.png", dpi=150); plt.close(fig)
print(res.round(4).to_string(index=False))