import sys
import numpy as np, pandas as pd
import matplotlib.pyplot as plt

df = pd.read_csv(sys.argv[1])
print(df.columns.tolist())

def pick(*names):
    for n in names:
        if n in df.columns:
            return n
    return None

t     = df[pick("t", "time")].to_numpy()
d     = df[pick("d", "lookahead", "d_la")].to_numpy()
cte   = df[pick("cte", "cte_true", "cross_track")].to_numpy()
kcol  = pick("kappa", "curvature")
pcol  = pick("psi", "steer", "delta")

dt = np.median(np.diff(t))
dd = np.diff(d) / dt                      # tốc độ thay đổi của d
sign_changes = np.sum(np.diff(np.sign(dd[np.abs(dd) > 0.05])) != 0)
print(f"d_mean={d.mean():.2f}  RMS(dd/dt)={np.sqrt(np.mean(dd**2)):.3f}  "
      f"đổi chiều={sign_changes}")

rows = [("d", d), ("|cte|", np.abs(cte))]
if kcol: rows.append(("kappa", df[kcol].to_numpy()))
if pcol: rows.append(("steer", df[pcol].to_numpy()))
fig, ax = plt.subplots(len(rows), 1, sharex=True, figsize=(10, 8))
for a, (name, y) in zip(ax, rows):
    a.plot(t, y); a.set_ylabel(name); a.grid(True)
ax[-1].set_xlabel("t (s)")
plt.tight_layout(); plt.show()