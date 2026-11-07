import argparse
import pandas as pd

ap = argparse.ArgumentParser()
ap.add_argument("seeds", nargs="*", type=int, default=[7, 21])
ap.add_argument("--part", default="fail")
a = ap.parse_args()

for lm in ("lm3_r4", "lm8_r4"):
    d = pd.read_csv(rf"results\giaidoan4_localization\diag_{a.part}_corridor_{lm}_high.csv")
    print("====", a.part, lm, d.shape)
    if "seed" not in d.columns:
        print("file này không có cột seed; các cột:", list(d.columns))
        continue
    print(d[d.seed.isin(a.seeds)].reset_index(drop=True).T.to_string())