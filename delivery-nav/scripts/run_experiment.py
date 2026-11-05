"""Tầng 1: thí nghiệm 2x2 (bộ bám: d cố định / thích nghi) x (định vị: dead reckoning / EKF).
Mỗi seed chạy cả BỐN tổ hợp trong cùng vòng lặp -> cùng chuỗi nhiễu odometry (so sánh ghép cặp).

    python scripts/run_experiment.py --map corridor --lm lm8 --seeds 30
    python scripts/run_experiment.py --map corridor --lm lm8 --seeds 5 --noise mid      # chạy thử nhanh
    python scripts/run_experiment.py --map mixed --loc dr --noise low                   # chỉ dead reckoning

Bộ thích nghi lấy từ run_tracking.ADAPT (đóng băng trước khi chạy). Đầu ra:
results/giaidoan5_experiment/exp_<map>_<lm>_d<d>[_tu<tau_up>].csv, bảng tổng hợp, kiểm định ghép cặp, boxplot.
"""
import argparse, sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT)); sys.path.insert(0, str(ROOT / "scripts"))
import numpy as np
import pandas as pd
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from scipy.stats import wilcoxon, binomtest
import run_tracking as rt

TRACKERS = ("fixed", "adaptive")
LOCS = ("dr", "ekf")


def run_one(path, junctions, clearance, name, tracker, loc, noise, seed, lm, d_fixed):
    sensor = rt.make_sensor(name, lm, seed) if loc == "ekf" else None
    r, _, _ = rt.run_episode(path, junctions, d_fixed, "geometric", clearance=clearance, loc=loc,
                             noise_level=noise, seed=seed, sensor=sensor,
                             adaptive=rt.ADAPT if tracker == "adaptive" else None)
    return dict(tracker=tracker, **r)


def wilcoxon_p(x, y):
    diff = (np.asarray(x) - np.asarray(y))
    diff = diff[~np.isnan(diff)]
    if len(diff) == 0 or not (diff != 0).any():
        return float("nan")
    try:
        return float(wilcoxon(diff).pvalue)
    except ValueError:
        return float("nan")


def mcnemar_p(a_ok, b_ok):
    """McNemar chính xác trên thành công nhị phân: b = chỉ A thành công, c = chỉ B thành công."""
    a_ok, b_ok = np.asarray(a_ok, bool), np.asarray(b_ok, bool)
    b, c = int((a_ok & ~b_ok).sum()), int((~a_ok & b_ok).sum())
    return (b, c, float(binomtest(min(b, c), b + c, 0.5).pvalue) if b + c else float("nan"))


def paired_table(df, factor, a, b, group_cols, metrics):
    """So sánh a với b của biến `factor` (tracker hoặc loc) trên cùng seed, theo từng nhóm group_cols."""
    rows = []
    for key, g in df.groupby(group_cols, observed=True):
        key = key if isinstance(key, tuple) else (key,)
        for m in metrics:
            p = g.pivot_table(index="seed", columns=factor, values=m)
            if not {a, b} <= set(p.columns):
                continue
            row = dict(zip(group_cols, key), metric=m, mean_a=p[a].mean(), mean_b=p[b].mean(),
                       median_diff=(p[a] - p[b]).median(), p_wilcoxon=wilcoxon_p(p[a], p[b]))
            if m == "success":
                _, _, row["p_wilcoxon"] = mcnemar_p(p[a] > 0.5, p[b] > 0.5)    # thành công: dùng McNemar
            rows.append(row)
    return pd.DataFrame(rows)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--map", default="corridor")
    ap.add_argument("--lm", default="lm8", choices=list(rt.LM_CONFIGS))
    ap.add_argument("--seeds", type=int, default=30)
    ap.add_argument("--noise", nargs="+", default=["low", "mid", "high"])
    ap.add_argument("--d", type=float, default=1.5, help="look-ahead cố định (d*)")
    ap.add_argument("--tau_up", type=float, default=None, help="ghi đè ADAPT['tau_up']")
    ap.add_argument("--loc", nargs="+", default=["dr", "ekf"], choices=["dr", "ekf"])
    a = ap.parse_args()
    if a.tau_up is not None:
        rt.ADAPT = {**rt.ADAPT, "tau_up": a.tau_up}
    path, junctions, clearance = rt.get_route(a.map)
    out_dir = ROOT / "results" / "giaidoan5_experiment"; out_dir.mkdir(parents=True, exist_ok=True)
    print("ADAPT =", rt.ADAPT, "| d cố định =", a.d, "| loc =", a.loc)

    rows = []
    for noise in a.noise:
        for seed in range(a.seeds):
            for tracker in TRACKERS:
                for loc in a.loc:
                    rows.append(dict(cfg=a.lm, **run_one(path, junctions, clearance, a.map, tracker, loc,
                                                         noise, seed, a.lm, a.d)))
        print(f"  xong nhiễu {noise}")
    df = pd.DataFrame(rows)
    df["noise"] = pd.Categorical(df["noise"], categories=a.noise, ordered=True)

    tag = f"_tu{a.tau_up:g}" if a.tau_up is not None else ""
    f_csv = out_dir / f"exp_{a.map}_{a.lm}_d{a.d:g}{tag}.csv"; df.to_csv(f_csv, index=False); print("Đã lưu", f_csv)

    metrics = ["success", "cte_rms", "cte_max", "miss_max", "steer_rate", "time",
               "loc_err_rms", "loc_err_max", "collisions"]
    g = df.groupby(["noise", "tracker", "loc"], observed=True)[metrics].mean().round(3)
    pd.set_option("display.width", 220, "display.max_columns", 40)
    print("\nTrung bình theo (nhiễu, bộ bám, định vị):\n", g.to_string())
    print("\nTrung vị cte_rms:\n", df.groupby(["noise", "tracker", "loc"], observed=True)["cte_rms"].median().round(3).to_string())

    pm = ["cte_rms", "cte_max", "steer_rate", "success"]
    print("\n[1] THÍCH NGHI so với CỐ ĐỊNH (a = adaptive, b = fixed; median_diff < 0 nghĩa là thích nghi nhỏ hơn):")
    print(paired_table(df, "tracker", "adaptive", "fixed", ["noise", "loc"], pm).to_string(index=False, float_format=lambda x: f"{x:.3g}"))
    if set(a.loc) >= {"dr", "ekf"}:
        pm2 = ["cte_rms", "cte_max", "success", "loc_err_max"]
        print("\n[2] EKF so với DEAD RECKONING (a = ekf, b = dr; median_diff < 0 nghĩa là EKF nhỏ hơn):")
        print(paired_table(df, "loc", "ekf", "dr", ["noise", "tracker"], pm2).to_string(index=False, float_format=lambda x: f"{x:.3g}"))

    fig, axs = plt.subplots(1, len(a.noise), figsize=(5 * len(a.noise), 4.5), squeeze=False, sharey=True)
    for ax, noise in zip(axs[0], a.noise):
        data, labels = [], []
        for tracker in TRACKERS:
            for loc in a.loc:
                s = df[(df.noise == noise) & (df.tracker == tracker) & (df["loc"] == loc)]
                data.append(s["cte_rms"].values); labels.append(f"{tracker}\n{loc}")
        ax.boxplot(data)
        ax.set_xticks(range(1, len(labels) + 1))
        ax.set_xticklabels(labels)
        ax.set_title(f"nhiễu {noise}"); ax.grid(alpha=.3)
    axs[0][0].set_ylabel("cte_rms (m), so với pose THẬT"); plt.tight_layout()
    f_png = out_dir / f"box_cte_{a.map}_{a.lm}_d{a.d:g}{tag}.png"; plt.savefig(f_png, dpi=130); print("Đã lưu", f_png)


if __name__ == "__main__":
    main()