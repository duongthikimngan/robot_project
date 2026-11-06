"""Tần suất trượt từng điểm giao, từ CSV do diag_lm3r4.py --part fail sinh ra.
    python scripts\\analyze_miss.py                                  # mặc định corridor / lm3_r4 / high
    python scripts\\analyze_miss.py --map corridor --lm lm8_r4 --noise high
"""
import argparse
from pathlib import Path
import pandas as pd

ROOT = Path(__file__).resolve().parents[1]
ap = argparse.ArgumentParser()
ap.add_argument("--map", default="corridor"); ap.add_argument("--lm", default="lm3_r4")
ap.add_argument("--noise", default="high"); ap.add_argument("--r-reach", type=float, default=1.0)
a = ap.parse_args()

f = ROOT / "results" / "giaidoan4_localization" / f"diag_fail_{a.map}_{a.lm}_{a.noise}.csv"
d = pd.read_csv(f)
mc = [c for c in d.columns if c.startswith("miss_j")]
if not mc:
    raise SystemExit("CSV chưa có cột miss_j*: thêm out.update(...) vào run_episode rồi chạy lại diag_lm3r4.py --part fail")
print(f"{f.name}: {len(d)} lượt")
print(f"\ntần suất trượt từng điểm (miss > R_REACH = {a.r_reach:g} m):")
print((d[mc] > a.r_reach).mean().round(2).to_string())
print("\ntách theo NEES nhất quán (False) / mất nhất quán (True):")
print((d[mc] > a.r_reach).groupby(d.nees_mean > 4.5).mean().round(2).to_string())