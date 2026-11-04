import sys, os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import numpy as np
import matplotlib.pyplot as plt
from src.maps import corridor_map
from src.astar import astar

m, junctions = corridor_map()
inf = m.inflate(1.0)

route, total = [], 0.0
for a, b in zip(junctions[:-1], junctions[1:]):
    p, c = astar(inf, a, b)
    if p is None:
        raise SystemExit(f"khong co duong {a}->{b}")
    route += p if not route else p[1:]      # bo diem noi trung nhau
    total += c
route = np.array(route)
print("so diem:", len(route), "| tong do dai:", round(total, 2), "m")

ext = [0, 50, 0, 50]
fig, ax = plt.subplots(figsize=(7, 7))
ax.imshow(inf.occ, origin="lower", extent=ext, cmap="Reds", alpha=0.4)
ax.imshow(m.occ, origin="lower", extent=ext, cmap="Greys", alpha=0.8)
ax.plot(route[:, 0], route[:, 1], "b.-", ms=3, lw=1)
for k, (x, y) in enumerate(junctions, 1):
    ax.plot(x, y, "go")
    ax.text(x + 0.5, y + 0.5, str(k))
ax.set_title("A* tho (chua lam muot): 6 diem giao")
plt.savefig(r"D:\robot_project\delivery-nav\results\astar_raw_corridor_2026-11-04.png", dpi=150)
plt.show()