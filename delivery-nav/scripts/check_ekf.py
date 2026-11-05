"""Giai đoạn 4: kiểm tra nhất quán và so sánh dead reckoning với EKF trên nhiều seed.

    python scripts/check_ekf.py --map corridor --seeds 30 --lm lm8
    python scripts/check_ekf.py --map corridor --seeds 30 --lm lm3 lm8 lm8_r4        # nhiều cấu hình, 1 file CSV
    python scripts/check_ekf.py --map corridor --phaseb --noise-b mid --seed-b 0     # Pha B: 1 seed, vẽ ảnh
    python scripts/check_ekf.py --demo --seeds 10                                    # thử khung bằng tuyến giả

Cả hai chế độ dùng CÙNG seed -> CÙNG chuỗi nhiễu odometry (so sánh ghép cặp).
Tiêu chí nhất quán: NEES trung bình trong khoảng 2-4,5 và >= ~90% mẫu có NEES < 7,81.
Đầu ra: CSV từng lượt (cột cfg, noise, loc, seed, ...), bảng tổng hợp, danh sách lượt EKF thất bại,
kiểm định Wilcoxon ghép cặp EKF so với DR theo từng (cfg, noise).
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
import run_tracking as rt


def one(path, junctions, clearance, name, loc, noise, seed, lm, adaptive):
    sensor = rt.make_sensor(name, lm, seed) if loc == "ekf" else None
    return rt.run_episode(path, junctions, 1.5, "geometric", clearance=clearance, loc=loc, noise_level=noise,
                          seed=seed, sensor=sensor, adaptive=rt.ADAPT if adaptive else None)


def phase_b(a, name, path, junctions, clearance, out_dir):
    lm_cfg = a.lm[0]
    res = {loc: one(path, junctions, clearance, name, loc, a.noise_b, a.seed_b, lm_cfg, a.adaptive)
           for loc in ("dr", "ekf")}
    fig, ax = plt.subplots(1, 2, figsize=(14, 6), gridspec_kw=dict(width_ratios=[1.3, 1]))
    ax[0].plot(*path.T, "k--", lw=1, label="tuyến tham chiếu")
    ax[0].plot(*res["ekf"][1].T, "b", lw=1.8, label="quỹ đạo THẬT (xe điều khiển bằng EKF)")
    ax[0].plot(res["ekf"][2][:, 5], res["ekf"][2][:, 6], "r", lw=1, label="ước lượng EKF")
    ax[0].plot(res["dr"][2][:, 5], res["dr"][2][:, 6], "g", lw=1, label="ước lượng dead reckoning (lượt riêng)")
    ax[0].plot(*junctions.T, "m*", ms=12, label="điểm giao")
    try:
        lm = rt.make_sensor(name, lm_cfg, a.seed_b).lm; ax[0].plot(*lm.T, "g^", label="landmark")
    except Exception:
        pass
    ax[0].set_aspect("equal"); ax[0].grid(alpha=.3); ax[0].legend(fontsize=8)
    ax[0].set_title(f"Pha B: {name}, nhiễu {a.noise_b}, seed {a.seed_b}, {lm_cfg}")
    for loc, c in (("dr", "g"), ("ekf", "r")):
        lg = res[loc][2]
        ax[1].plot(lg[:, 0], lg[:, 7], c, label=f"sai số định vị {loc.upper()} (m)")
        ax[1].plot(lg[:, 0], 2 * lg[:, 8], c + ":", label=f"2·σ_xy {loc.upper()}")
    ax[1].set_xlabel("t (s)"); ax[1].grid(alpha=.3); ax[1].legend(fontsize=8)
    ax[1].set_title("Sai số thật so với dải ±2σ từ ma trận P")
    plt.tight_layout()
    f = out_dir / f"phaseB_{name}_{a.noise_b}_seed{a.seed_b}_{lm_cfg}{'_adaptive' if a.adaptive else ''}.png"
    plt.savefig(f, dpi=130); print("Đã lưu", f)
    for loc in ("dr", "ekf"):
        r = res[loc][0]
        print(loc, {k: r[k] for k in ("success", "loc_err_end", "loc_err_rms", "nees_mean", "nees_in95", "n_obs", "n_rej")})


def paired_tests(df, metric="loc_err_rms"):
    """Wilcoxon ghép cặp EKF so với DR (cùng seed, cùng chuỗi nhiễu odometry) theo từng (cfg, noise)."""
    from scipy.stats import wilcoxon
    rows = []
    for (cfg, noise), g in df.groupby(["cfg", "noise"], observed=True):
        p = g.pivot_table(index="seed", columns="loc", values=metric)
        if not {"dr", "ekf"} <= set(p.columns):
            continue
        diff = (p["dr"] - p["ekf"]).dropna()
        try:
            pv = wilcoxon(diff).pvalue if (diff != 0).any() else float("nan")
        except ValueError:
            pv = float("nan")
        rows.append(dict(cfg=cfg, noise=noise, n=len(diff), dr_mean=p["dr"].mean(), ekf_mean=p["ekf"].mean(),
                         median_gain=diff.median(), ekf_better_frac=(diff > 0).mean(), p_wilcoxon=pv))
    return pd.DataFrame(rows).round(4)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--map", default="corridor")
    ap.add_argument("--demo", action="store_true")
    ap.add_argument("--seeds", type=int, default=30)
    ap.add_argument("--noise", nargs="+", default=["low", "mid", "high"])
    ap.add_argument("--lm", nargs="+", default=["lm8"], choices=list(rt.LM_CONFIGS),
                    help="một hoặc nhiều cấu hình landmark")
    ap.add_argument("--adaptive", action="store_true", help="dùng look-ahead thích nghi thay cho d = 1,5")
    ap.add_argument("--phaseb", action="store_true")
    ap.add_argument("--noise-b", default="mid"); ap.add_argument("--seed-b", type=int, default=0)
    a = ap.parse_args()
    name = "demo" if a.demo else a.map
    path, junctions, clearance = rt.demo_route() if a.demo else rt.get_route(a.map)
    out_dir = ROOT / "results" / "giaidoan4_localization"; out_dir.mkdir(parents=True, exist_ok=True)

    if a.phaseb:
        phase_b(a, name, path, junctions, clearance, out_dir)
        return

    rows = []
    for lm_cfg in a.lm:
        for noise in a.noise:
            for seed in range(a.seeds):
                for loc in ("dr", "ekf"):
                    r, _, _ = one(path, junctions, clearance, name, loc, noise, seed, lm_cfg, a.adaptive)
                    rows.append(dict(cfg=lm_cfg, **r))
            print(f"  xong {lm_cfg}, nhiễu {noise}")
    df = pd.DataFrame(rows)
    df["noise"] = pd.Categorical(df["noise"], categories=a.noise, ordered=True)
    suffix = "_adaptive" if a.adaptive else ""
    csv_path = out_dir / f"ekf_check_{name}_{'-'.join(a.lm)}{suffix}.csv"
    df.to_csv(csv_path, index=False)
    print("Đã lưu", csv_path)

    agg = dict(n=("seed", "count"), success=("success", "mean"),
               loc_err_end=("loc_err_end", "mean"), loc_err_rms=("loc_err_rms", "mean"),
               nees_mean=("nees_mean", "mean"), nees_in95=("nees_in95", "mean"),
               n_obs=("n_obs", "mean"), n_rej=("n_rej", "mean"), collisions=("collisions", "mean"))
    if "loc_err_max" in df.columns:                      # nếu run_episode có trả về sai số lớn nhất
        agg["loc_err_max"] = ("loc_err_max", "mean")
    g = df.groupby(["cfg", "noise", "loc"], observed=True).agg(**agg).round(3)
    pd.set_option("display.width", 220, "display.max_columns", 30)
    print("\n", g.to_string())

    ok = g.nees_mean.between(2.0, 4.5) & (g.nees_in95 >= 0.90)
    print("\nNhất quán (NEES TB trong [2; 4,5] và >= 90% mẫu < 7,81):")
    print(ok.to_string())

    fail = df[(df["loc"] == "ekf") & (df["success"] == 0)]
    cols = [c for c in ("cfg", "noise", "seed", "n_reached", "missed", "collisions", "loc_err_rms", "loc_err_max",
                        "th_err_max", "rej_frac", "t_err1", "n_rej", "nees_mean") if c in df.columns]
    print(f"\nLượt EKF thất bại: {len(fail)}/{(df['loc'] == 'ekf').sum()}")
    if len(fail):
        print(fail[cols].to_string(index=False))

    print("\nWilcoxon ghép cặp (DR trừ EKF, loc_err_rms; median_gain > 0 nghĩa là EKF tốt hơn):")
    print(paired_tests(df).to_string(index=False))


if __name__ == "__main__":
    main()