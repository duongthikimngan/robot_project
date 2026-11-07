import argparse
import pandas as pd

ap = argparse.ArgumentParser()
ap.add_argument("--part", default="fail")
ap.add_argument("--rreach", type=float, default=1.0)
ap.add_argument("--rej", type=float, default=0.3, help="ngưỡng tỉ lệ quan sát bị loại để coi là mất khóa")
ap.add_argument("--nees", type=float, default=4.5)
a = ap.parse_args()

for lm in ("lm3_r4", "lm8_r4"):
    d = pd.read_csv(rf"results\giaidoan4_localization\diag_{a.part}_corridor_{lm}_high.csv").set_index("seed")
    mc = [c for c in d.columns if c.startswith("miss_j")]
    d["n_miss"] = (d[mc] > a.rreach).sum(axis=1)
    d["loai"] = "thành công"
    d.loc[d.n_miss > 0, "loai"] = "lỡ điểm"
    d.loc[d.collisions > 0, "loai"] = "va chạm"
    d["mat_khoa"] = d.rej_frac > a.rej
    d["nq"] = d.nees_mean > a.nees

    print(f"\n==== {lm}: {len(d)} lượt")
    print("tỉ lệ quan sát bị loại, 8 lượt cao nhất (tìm khoảng trống tự nhiên để chọn ngưỡng):")
    print(d.rej_frac.sort_values(ascending=False).head(8).round(2).to_string())
    print(f"\nkết quả theo mất khóa (rej_frac > {a.rej}):")
    print(pd.crosstab(d.loai, d.mat_khoa, margins=True).to_string())
    print(f"\nmất khóa theo mất nhất quán (NEES > {a.nees}):")
    print(pd.crosstab(d.mat_khoa, d.nq, margins=True).to_string())
    print("\nđiểm trượt theo mất khóa:")
    print(d.groupby("mat_khoa").n_miss.agg(["count", "sum", "mean"]).round(2).to_string())
    cols = ["loai", "n_miss", "nees_mean", "rej_frac", "loc_err_end", "collisions", "th_err_max"]
    print("\ncác lượt mất nhất quán, kèm chỉ số cơ chế:")
    print(d.loc[d.nq, cols].round(2).to_string())
    print("\ncác lượt có va chạm:")
    print(d.loc[d.collisions > 0, cols].round(2).to_string())