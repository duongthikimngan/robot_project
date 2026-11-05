import sys
from pathlib import Path
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT)); sys.path.insert(0, str(ROOT / "scripts"))
import numpy as np
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt
import run_tracking as rt

path, junctions, clearance = rt.get_route("corridor")
NOISE, SEED, LM = "mid", 0, "lm8"
P = np.asarray(path, float)[:, :2]

def run(adaptive):
    sensor = rt.make_sensor("corridor", LM, SEED)
    return rt.run_episode(path, junctions, 1.5, "geometric", clearance=clearance,
                          loc="ekf", noise_level=NOISE, seed=SEED, sensor=sensor,
                          adaptive=rt.ADAPT if adaptive else None)

def cte_polyline(xy):
    A, B = P[:-1], P[1:]
    AB = B - A
    L2 = np.maximum((AB ** 2).sum(1), 1e-12)
    out = np.empty(len(xy))
    for i, p in enumerate(xy):
        t = np.clip(((p - A) * AB).sum(1) / L2, 0, 1)
        out[i] = np.min(np.hypot(*(p - (A + t[:, None] * AB)).T))
    return out

def analyse(adaptive):
    r, xy, lg = run(adaptive)
    xy = np.asarray(xy)[:, :2]
    c = cte_polyline(xy)
    s = np.r_[0, np.cumsum(np.hypot(*np.diff(xy, axis=0).T))]
    print(("thich nghi" if adaptive else "d=1.5   "),
          "| r: rms %.3f max %.3f | tu tinh: rms %.3f max %.3f" %
          (r["cte_rms"], r["cte_max"], np.sqrt((c**2).mean()), c.max()))
    return r, s, c, np.asarray(lg)

rA, sA, cA, lgA = analyse(True)
rF, sF, cF, lgF = analyse(False)

# tim cot d trong log thich nghi: mean ~ d_mean, min ~ d_min
dcol = None
for j in range(lgA.shape[1]):
    col = lgA[:, j]
    if abs(np.nanmean(col) - rA["d_mean"]) < 0.05 and abs(np.nanmin(col) - rA["d_min"]) < 0.05:
        dcol = j; print("cot d =", j); break
if dcol is None:
    print("Khong tim thay cot d; in vai dong dau de xem:"); print(np.round(lgA[:3], 3))

fig, ax = plt.subplots(2, 1, figsize=(11, 6), sharex=True)
ax[0].plot(sF, cF, label="d = 1.5 cố định")
ax[0].plot(sA, cA, label="thích nghi")
ax[0].set_ylabel("cte (m)"); ax[0].legend(); ax[0].grid(alpha=.3)
if dcol is not None:
    n = min(len(sA), len(lgA))
    ax[1].plot(sA[:n], lgA[:n, dcol], color="C1")
ax[1].set_ylabel("d thích nghi (m)"); ax[1].set_xlabel("quãng đường (m)"); ax[1].grid(alpha=.3)
plt.tight_layout()
out = ROOT / "results" / "giaidoan5_experiment" / "cte_vs_s_mid_seed0.png"
plt.savefig(out, dpi=150); print("Đã lưu", out)

import copy
for tu in (1.0, 2.0, 4.0, 8.0):
    cfg = copy.deepcopy(rt.ADAPT); cfg["tau_up"] = tu
    sensor = rt.make_sensor("corridor", LM, SEED)
    r, _, _ = rt.run_episode(path, junctions, 1.5, "geometric", clearance=clearance,
                             loc="ekf", noise_level=NOISE, seed=SEED, sensor=sensor, adaptive=cfg)
    print(f"tau_up={tu}: cte_rms {r['cte_rms']:.3f} cte_max {r['cte_max']:.3f} "
          f"miss_max {r['miss_max']:.3f} steer_rate {r['steer_rate']:.4f}")
