import sys
import pandas as pd

lm = sys.argv[1] if len(sys.argv) > 1 else "lm8_r4"
noise = sys.argv[2] if len(sys.argv) > 2 else "high"
seeds = [int(x) for x in sys.argv[3:]] or [7]

dfs = {}
for g in ("9.21", "13.82", "off"):
    p = rf"results\giaidoan4_localization\diag_gate_seeds_corridor_{lm}_{noise}_{g}.csv"
    dfs[g] = pd.read_csv(p).set_index("seed")

succ = pd.DataFrame({g: d.success for g, d in dfs.items()})
print(f"==== {lm} {noise}: số lượt thành công theo cổng")
print(succ.sum().to_string())

base = succ["9.21"]
chg = succ[(succ["13.82"] != base) | (succ["off"] != base)]
print("\ncác seed đổi kết quả so với cổng mặc định 9.21:")
print(chg.to_string())

cols = [c for c in ("success", "n_reached", "loc_err_end", "n_obs", "n_rej",
                    "nees_mean", "collisions") if c in dfs["9.21"].columns]
for s in seeds:
    print(f"\nseed {s}:")
    print(pd.DataFrame({g: d.loc[s, cols] for g, d in dfs.items()}).to_string())