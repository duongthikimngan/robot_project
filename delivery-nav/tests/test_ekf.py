import sys, os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import numpy as np
from src.ekf import EKF, motion, Fx, Fv, obs, Hx, wrap, nees

ok = True
def check(name, cond, info=""):
    global ok; ok &= bool(cond); print("PASS " if cond else "FAIL ", name, info)

def numjac(fun, x, eps=1e-6):
    f0 = fun(x); J = np.zeros((len(f0), len(x)))
    for i in range(len(x)):
        d = np.zeros(len(x)); d[i] = eps
        J[:, i] = (fun(x + d) - fun(x - d)) / (2 * eps)
    return J

rng = np.random.default_rng(0)
# 1) Jacobian giải tích khớp Jacobian số (kiểm thử quan trọng nhất)
for _ in range(20):
    q = np.array([rng.uniform(-5, 5), rng.uniform(-5, 5), rng.uniform(-3, 3)])
    u = np.array([rng.uniform(0, 0.3), rng.uniform(-0.1, 0.1)])
    p = q[:2] + rng.uniform(2, 8, 2) * rng.choice([-1, 1], 2)
    e1 = np.abs(Fx(q, u) - numjac(lambda x: motion(x, u), q)).max()
    e2 = np.abs(Fv(q, u) - numjac(lambda v: motion(q, v), u)).max()
    def h_(x):
        z = obs(x, p); return z
    e3 = np.abs(Hx(q, p) - numjac(h_, q)).max()
    if max(e1, e2, e3) > 1e-5: break
check("Jacobian Fx, Fv, Hx khớp Jacobian số", max(e1, e2, e3) < 1e-5, f"sai lệch lớn nhất {max(e1,e2,e3):.1e}")
# 2) chuẩn hóa góc quanh ±pi
q = np.array([0.0, 0.0, np.pi - 0.01]); p = np.array([-5.0, -0.1])
nu = obs(q, p) - obs(q + np.array([0, 0, 0.02]), p); nu[1] = wrap(nu[1])
check("innovation không nhảy 2π quanh ±π", abs(nu[1]) < 0.05, f"bearing lệch {nu[1]:.3f}")
# 3) không nhiễu: EKF phải bám đúng tư thế thật
V0 = np.zeros((2, 2)); W = np.diag([0.1**2, np.deg2rad(1)**2])
q_true = np.array([0.0, 0.0, 0.0]); ekf = EKF(q_true, np.eye(3) * 1e-4, W); lm = np.array([[5, 5], [-5, 5], [0, -6.0]])
for k in range(200):
    u = np.array([0.1, 0.02]); q_true = motion(q_true, u); ekf.predict(u, V0)
    for p in lm: ekf.update(obs(q_true, p), p)
check("không nhiễu: EKF = tư thế thật", np.abs(ekf.q - q_true).max() < 1e-6, f"sai lệch {np.abs(ekf.q-q_true).max():.1e}")
# 4) không thấy landmark nào => EKF trùng dead reckoning
a = EKF(np.zeros(3), np.eye(3) * 1e-4, W); q_dr = np.zeros(3)
for k in range(100):
    u = np.array([0.1, 0.01]); a.predict(u, np.diag([1e-4, 1e-5])); q_dr = motion(q_dr, u)
check("không có quan sát => trùng dead reckoning", np.abs(a.q - q_dr).max() < 1e-12)
# 5) quan sát làm giảm bất định; P đối xứng, xác định dương
b = EKF(np.zeros(3), np.diag([1.0, 1.0, 0.1]), W); tr0 = np.trace(b.P)
b.update(obs(np.zeros(3), np.array([6.0, 2.0])), np.array([6.0, 2.0]))
ev = np.linalg.eigvalsh(b.P)
check("update giảm vết P, P đối xứng và dương", np.trace(b.P) < tr0 and ev.min() > 0 and np.allclose(b.P, b.P.T))
# 6) cổng: quan sát sai lớn bị loại; landmark quá gần bị bỏ
c = EKF(np.zeros(3), np.eye(3) * 1e-3, W)
check("gating loại quan sát sai", c.update(obs(np.zeros(3), np.array([6.0, 2.0])) + np.array([3.0, 0.0]), np.array([6.0, 2.0])) is False)
check("bỏ landmark quá gần", c.update(np.array([0.5, 0.0]), np.array([0.5, 0.0])) is False)
# 7) nhất quán (NEES) trên nhiều seed, nhiễu odometry tăng theo quãng đường
cd, cth = 0.03, np.deg2rad(0.285)
def sim(seed, T=600, with_lm=True):
    r = np.random.default_rng(seed); rs = np.random.default_rng(seed + 10_000)
    qt = np.array([10.0, 0.0, np.pi / 2]); e = EKF(qt, np.diag([0.05**2, 0.05**2, np.deg2rad(0.5)**2]), W)
    L = np.array([[10, 0], [0, 10], [-10, 0], [0, -10], [8, 8], [-8, -8.0]]); ns = []
    for k in range(T):
        u = np.array([0.1, 0.1 / 10.0]); qt = motion(qt, u)
        um = u + np.array([r.normal(0, cd * np.sqrt(u[0])), r.normal(0, cth * np.sqrt(u[0]))])
        e.predict(um, np.diag([cd**2 * u[0], cth**2 * u[0]]))           # V đúng bằng nhiễu thật
        if with_lm and k % 2 == 0:
            for p in L:
                if np.hypot(*(p - qt[:2])) < 8: e.update(obs(qt, p) + rs.normal(0, [0.1, np.deg2rad(1)]), p)
        ns.append(nees(e.q.copy(), qt, e.P))
    return np.array(ns), np.hypot(*(e.q[:2] - qt[:2]))
res = [sim(s) for s in range(30)]; allv = np.concatenate([r[0] for r in res])
check("NEES trung bình ~ 3 (bộ lọc nhất quán)", 2.0 < allv.mean() < 4.5, f"NEES TB = {allv.mean():.2f}, trong chi2 95% (<7.81): {100*np.mean(allv<7.81):.1f}%")
dr = [sim(s, with_lm=False)[1] for s in range(30)]
check("EKF tốt hơn dead reckoning rõ rệt", np.mean([r[1] for r in res]) < 0.2 * np.mean(dr),
      f"sai số cuối: EKF {np.mean([r[1] for r in res]):.2f} m, DR {np.mean(dr):.2f} m")
print("\nTẤT CẢ PASS" if ok else "\nCÓ LỖI")