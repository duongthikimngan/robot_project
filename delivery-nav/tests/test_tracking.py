import sys, os
sys.path.insert(0, os.path.dirname(os.path.dirname(__file__)))
import numpy as np
from src.vehicle import Bicycle
from src.pursuit import PurePursuit
from src.pid import PID, simc_pi

ok = True
def check(name, cond, info=""):
    global ok; ok &= bool(cond); print(("PASS " if cond else "FAIL "), name, info)

# 1) vòng tròn bán kính L/tan(psi)
for L, psi in [(1.0, 0.4), (0.8, 0.6), (2.5, 0.3)]:
    car = Bicycle((0, 0, 0), L=L, psi_max=1.0, dt=0.05)
    pts = []
    for _ in range(int(2 * np.pi * (L / np.tan(psi)) / 1.0 / 0.05) + 2):
        pts.append(car.step(1.0, psi)[0][:2])
    pts = np.array(pts); R = L / np.tan(psi)
    c = np.array([0, R]); r = np.hypot(*(pts - c).T)
    check(f"vòng tròn L={L} psi={psi}", abs(r.mean() - R) < 0.01 * R, f"R_lý_thuyết={R:.3f} R_đo={r.mean():.3f}")
# 2) kẹp góc lái
car = Bicycle(L=1.0, psi_max=0.5); _, p = car.step(1.0, 2.0)
check("kẹp psi_max", abs(p - 0.5) < 1e-12)

def drive(path, q0, d, law, L=1.0, psi_max=0.6, v=1.0, T=120):
    car = Bicycle(q0, L=L, psi_max=psi_max, dt=0.1); pp = PurePursuit(path, L, psi_max, d=d, law=law)
    cte = []
    for _ in range(int(T / 0.1)):
        psi, _, _ = pp(car.q); car.step(v, psi); cte.append(pp.cross_track(car.q[:2]))
        if pp.finished(car.q[:2]): break
    return np.array(cte)

# 3) đường thẳng: xe lệch 2 m phải hội tụ
line = np.array([[0, 0], [80, 0]], float)
for law in ["geometric", "slide"]:
    c = drive(line, (0, 2.0, 0.0), d=3.0, law=law)
    check(f"đường thẳng, law={law}", c[-50:].max() < 0.05, f"CTE cuối={c[-50:].max():.4f}")
# 4) vòng tròn R=10: sai số ổn định
th = np.linspace(0, 2 * np.pi * 1.3, 800); circ = np.stack([10 * np.cos(th) - 10, 10 * np.sin(th)], 1)
for law in ["geometric", "slide"]:
    c = drive(circ, (0, 0, np.pi / 2), d=3.0, law=law, T=150)
    h = c[len(c) // 3: 2 * len(c) // 3]                      # đoạn giữa, đã qua quá độ
    tol = 0.05 if law == "geometric" else 0.5                # luật 'slide' (P thuần) có sai số xác lập khi đi cong
    check(f"vòng tròn R=10, law={law}", np.ptp(h) < 0.05 and h.max() < tol,
          f"CTE ổn định ~{h.mean():.3f} m (dao động {np.ptp(h):.3f})")
# 5) PID vận tốc bằng S-IMC trên quá trình bậc nhất có trễ
K, T, tau, Tc, dt = 1.0, 0.5, 0.1, 0.3, 0.05
kp, ti = simc_pi(K, T, tau, Tc); pid = PID(kp, ti, 0.0, dt, 0.0, 2.0)
v, buf, vs = 0.0, [0.0] * int(round(tau / dt)), []
for k in range(int(6 / dt)):
    u = pid(1.0, v); buf.append(u); u_d = buf.pop(0)
    v += dt * (K * u_d - v) / T; vs.append(v)
vs = np.array(vs)
check("PID S-IMC: không quá điều chỉnh >10%, xác lập về 1", vs.max() < 1.10 and abs(vs[-1] - 1) < 0.01,
      f"Kp={kp:.2f} Ti={ti:.2f} overshoot={100*(vs.max()-1):.1f}%")
# 6) d = 1/5/10 trên đường lệch 2 m, L = 1 và L = 2.5 (quan sát)
print("\nQuan sát: CTE trong 10 s cuối (đường thẳng, lệch 2 m, v=1 m/s; d và L khác nhau)")
for L in [1.0, 2.5]:
    for d in [1, 5, 10]:
        for law in ["geometric", "slide"]:
            c = drive(line, (0, 2.0, 0.0), d=d, law=law, L=L, T=80)
            print(f"  L={L} d={d:2d} {law:9s}: CTE cuối {c[-200:].max():.3f} m, dao động {np.ptp(c[-200:]):.3f}")
print("\nTẤT CẢ PASS" if ok else "\nCÓ LỖI")
