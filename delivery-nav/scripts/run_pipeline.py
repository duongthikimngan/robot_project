import sys, os, math
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import numpy as np
import matplotlib.pyplot as plt
from src.maps import maps
from src.astar import astar
from src.smooth import shortcut, resample

R_ROBOT = 1.0
name = sys.argv[1]                       # corridor | open
m, J = getattr(maps, f"{name}_map")()
inf = m.inflate(R_ROBOT)

for k, (x, y) in enumerate(J, 1):
    assert inf.is_free(*inf.to_grid(x, y)), f"diem giao {k} {(x, y)} nam trong vung phong"

raw, sm, total_raw, total_s = [], [], 0.0, 0.0
for a, b in zip(J[:-1], J[1:]):
    p, c = astar(inf, a, b)
    assert p is not None, f"khong co duong {a}->{b}"
    p = [tuple(a)] + p[1:-1] + [tuple(b)]
    r = resample(shortcut(inf, p), 0.5)
    assert all(inf.is_free(*inf.to_grid(x, y)) for x, y in r), "diem lam muot cham vat can"
    total_raw += c
    total_s += float(np.sum(np.hypot(*np.diff(r, axis=0).T)))
    raw += p
    sm += list(r)

print(f"[{name}] PASS | tho {total_raw:.2f} m -> muot {total_s:.2f} m | diem: {len(raw)} -> {len(sm)}")
dist_obs = [m_d for m_d in
            __import__('scipy.ndimage', fromlist=['x']).distance_transform_edt(~m.occ)
            [[int(y / m.res) for x, y in sm], [int(x / m.res) for x, y in sm]] * m.res]
print(f"khoang cach toi vat can that: min {min(dist_obs):.2f} m")

R = np.array(sm); Rw = np.array(raw)
ext = [0, 50, 0, 50]
fig, ax = plt.subplots(figsize=(7, 7))
ax.imshow(inf.occ, origin="lower", extent=ext, cmap="Reds", alpha=0.4)
ax.imshow(m.occ, origin="lower", extent=ext, cmap="Greys", alpha=0.8)
ax.plot(Rw[:, 0], Rw[:, 1], "c-", lw=1, label="A* tho")
ax.plot(R[:, 0], R[:, 1], "b.-", ms=3, lw=1, label="lam muot")
for k, (x, y) in enumerate(J, 1):
    ax.plot(x, y, "go"); ax.text(x + 0.5, y + 0.5, str(k))
ax.legend(); ax.set_title(f"{name}_map")
plt.savefig(rf"D:\robot_project\delivery-nav\results\pipeline_{name}_2026-11-04.png", dpi=150)
plt.show()