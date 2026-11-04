import sys, os, math
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import numpy as np
from src import maps
from src.planner import plan, reached

ok = True
def check(name, cond):
    global ok
    ok &= bool(cond)
    print(("PASS " if cond else "FAIL ") + name)

for name in ["corridor", "open", "mixed"]:
    m, J = getattr(maps, f"{name}_map")()
    W, idx = plan(m, J)
    inf = m.inflate(1.0)
    check(f"{name}: moi waypoint tu do",
          all(inf.is_free(*inf.to_grid(x, y)) for x, y in W))
    check(f"{name}: waypoint tai diem giao khop toa do",
          all(math.dist(W[i], j) < 1e-6 for i, j in zip(idx, J)))
    check(f"{name}: khoang cach lien tiep <= 0.5 m",
          np.max(np.hypot(*np.diff(W, axis=0).T)) <= 0.5 + 1e-9)

check("reached trong 0.5 m", reached((1.0, 1.0, 0.0), (1.3, 1.0)))
check("reached ngoai 0.5 m", not reached((1.0, 1.0, 0.0), (1.6, 1.0)))

try:
    m, J = maps.corridor_map()
    plan(m, [J[0], (10, 10)])        # (10,10) nam trong khoi vat can
    check("diem giao trong vat can phai nem loi", False)
except ValueError:
    check("diem giao trong vat can nem ValueError", True)

print("TAT CA PASS" if ok else "CO LOI")