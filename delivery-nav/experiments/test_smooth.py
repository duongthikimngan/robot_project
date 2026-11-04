import sys, os, math
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import numpy as np
import matplotlib.pyplot as plt
from src.maps import corridor_map
from src.astar import astar
from src.smooth import shortcut, resample, line_free

m, J = corridor_map()
inf = m.inflate(1.0)

raw, smooth_all, total_raw, total_s = [], [], 0.0, 0.0
for a, b in zip(J[:-1], J[1:]):
    p, c = astar(inf, a, b)
    p = [tuple(a)] + p[1:-1] + [tuple(b)]   # ghim 2 dau ve diem giao chinh xac
    s = shortcut(inf, p)
    r = resample(s, 0.5)
    total_raw += c
    total_s += float(np.sum(np.hypot(*np.diff(r, axis=0).T)))
    assert all(inf.is_free(*inf.to_grid(x, y)) for x, y in r), "diem lam muot cham vat can"
    assert math.dist(r[0], a) < 1e-6 and math.dist(r[-1], b) < 1e-6, "mat diem dau/cuoi"
    raw += p; smooth_all += list(r)
print("PASS: moi diem sau lam muot tu do, giu diem giao")
print(f"do dai tho {total_raw:.2f} m -> sau lam muot {total_s:.2f} m")
print("so diem tho:", len(raw), "-> sau:", len(smooth_all))

R = np.array(smooth_all)
ext = [0, 50, 0, 50]
fig, ax = plt.subplots(figsize=(7, 7))
ax.imshow(inf.occ, origin="lower", extent=ext, cmap="Reds", alpha=0.4)
ax.imshow(m.occ, origin="lower", extent=ext, cmap="Greys", alpha=0.8)
ax.plot(np.array(raw)[:, 0], np.array(raw)[:, 1], "c-", lw=1, label="A* tho")
ax.plot(R[:, 0], R[:, 1], "b.-", ms=3, lw=1, label="lam muot")
ax.legend()
plt.savefig(r"D:\robot_project\delivery-nav\results\astar_smooth_corridor_2026-11-04.png", dpi=150)
plt.show()