import sys
from pathlib import Path
import pandas as pd

ROOT = Path(__file__).resolve().parents[1]
f = Path(sys.argv[1]) if len(sys.argv) > 1 else ROOT / "results" / "giaidoan5_experiment" / "exp_corridor_lm8.csv"
d = pd.read_csv(f)
print(f.name)
print(d[d.tracker == "adaptive"].groupby(["noise", "loc"])[["d_mean", "d_min"]].mean().round(2))