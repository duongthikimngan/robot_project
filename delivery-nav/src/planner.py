import math
import numpy as np
from src.astar import astar
from src.smooth import shortcut, resample


def plan(gmap, junctions, r_robot=1.0, ds=0.5):
    """Pipeline Giai doan 2. Tra ve (waypoints Nx2, idx_junction): idx_junction[k] la chi so
    waypoint cua diem giao k. Nem ValueError neu khong co duong."""
    inf = gmap.inflate(r_robot)
    for k, j in enumerate(junctions, 1):
        if not inf.is_free(*inf.to_grid(*j)):
            raise ValueError(f"diem giao {k} {j} nam trong vung phong")
    wps, idx = [], [0]
    for a, b in zip(junctions[:-1], junctions[1:]):
        p, _ = astar(inf, a, b)
        if p is None:
            raise ValueError(f"khong co duong {a}->{b}")
        p = [tuple(a)] + p[1:-1] + [tuple(b)]
        r = resample(shortcut(inf, p), ds)
        wps += list(r) if not wps else list(r[1:])
        idx.append(len(wps) - 1)
    return np.array(wps), idx


def reached(pose_that, junction, r=0.5):
    """Den diem giao khi pose THAT (x, y, ...) cach junction < r (m)."""
    return math.hypot(pose_that[0] - junction[0], pose_that[1] - junction[1]) < r