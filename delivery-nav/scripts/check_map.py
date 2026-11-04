import sys, os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import numpy as np
import matplotlib.pyplot as plt
from src.maps import corridor_map

R_ROBOT = 1.0
m, junctions = corridor_map()
inf = m.inflate(R_ROBOT)

ext = [0, 50, 0, 50]
fig, ax = plt.subplots(figsize=(7, 7))
ax.imshow(inf.occ, origin="lower", extent=ext, cmap="Reds", alpha=0.4)
ax.imshow(m.occ, origin="lower", extent=ext, cmap="Greys", alpha=0.8)
for k, (x, y) in enumerate(junctions, 1):
    ok = inf.is_free(*inf.to_grid(x, y))
    ax.plot(x, y, "bo" if ok else "rx")
    ax.text(x + 0.5, y + 0.5, str(k))
    print("diem giao", k, (x, y), "tu do sau phong:", ok)
ax.set_title(f"corridor_map, res={m.res} m, phong {R_ROBOT} m")
plt.savefig(r"D:\robot_project\delivery-nav\results\map_corridor_2026-11-03.png", dpi=150)
plt.show()