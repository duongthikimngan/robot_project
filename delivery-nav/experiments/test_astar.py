import sys, os, math
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from src.gridmap import GridMap
from src.astar import astar
from src.maps import corridor_map

ok_all = True

def check(name, cond):
    global ok_all
    ok_all &= bool(cond)
    print(("PASS " if cond else "FAIL ") + name)

# 1. Ban do trong: duong cheo va duong thang co chi phi biet truoc
m = GridMap(20.0, 0.5)
p, c = astar(m, (1, 1), (9, 9))
check(f"cheo: cost={c:.3f}, ky vong {16 * math.sqrt(2) * 0.5:.3f}",
      abs(c - 16 * math.sqrt(2) * 0.5) < 1e-6)
p, c = astar(m, (1, 1), (9, 1))
check(f"thang: cost={c:.3f}, ky vong 8.000", abs(c - 8.0) < 1e-6)

# 2. Tuong chan kin tu bien den bien -> khong co duong
m = GridMap(20.0, 0.5)
m.add_rect(0, 10, 20, 11)
p, c = astar(m, (2, 2), (2, 18))
check("tuong chan kin -> None", p is None)

# 3. Diem dau hoac cuoi nam trong vat can -> None
m = GridMap(20.0, 0.5)
m.add_rect(5, 5, 6, 6)
p, c = astar(m, (5.2, 5.2), (15, 15))
check("start trong vat can -> None", p is None)

# 4. Co khe ho thi di qua, khe dong thi None
m = GridMap(20.0, 0.5)
m.add_rect(0, 10, 9, 11)
m.add_rect(11, 10, 20, 11)
p, c = astar(m, (2, 2), (2, 18))
check(f"co khe 2 m -> co duong (cost={c})", p is not None)

# 5. Ban do hanh lang da phong: moi o tren duong di phai tu do, do dai >= khoang cach thang
mp, junctions = corridor_map()
inf = mp.inflate(1.0)
for k in range(len(junctions) - 1):
    a, b = junctions[k], junctions[k + 1]
    p, c = astar(inf, a, b)
    straight = math.dist(a, b)
    free = all(inf.is_free(*inf.to_grid(x, y)) for x, y in p)
    check(f"giao {k+1}->{k+2}: cost={c:.2f} >= thang {straight:.2f}, o tu do={free}",
          p is not None and c >= straight - 1e-9 and free)

print("TAT CA PASS" if ok_all else "CO LOI")