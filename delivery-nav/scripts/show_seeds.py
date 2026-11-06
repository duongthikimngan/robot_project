import sys
import pandas as pd

seeds = [int(s) for s in sys.argv[1:]] or [7, 21]
for lm in ("lm3_r4", "lm8_r4"):
    d = pd.read_csv(rf"results\giaidoan4_localization\diag_fail_corridor_{lm}_high.csv").set_index("seed")
    print("====", lm)
    print(d.loc[seeds].T.to_string())