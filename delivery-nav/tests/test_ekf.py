"""Kiểm thử EKF. Chạy:  python tests\\test_ekf.py   (hoặc pytest tests\\test_ekf.py)"""
import sys, os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import numpy as np
from src.ekf import EKF, motion, Fx, Fv, obs, Hx, wrap, nees
from src.noise import LEVELS, measure, sigmas, dead_reckon_step

W = np.diag([0.1 ** 2, np.deg2rad(1.0) ** 2])


def numjac(fun, x, eps=1e-6):
    f0 = fun(x); J = np.zeros((len(f0), len(x)))
    for i in range(len(x)):
        d = np.zeros(len(x)); d[i] = eps
        J[:, i] = (fun(x + d) - fun(x - d)) / (2 * eps)
    return J


def test_jacobians():
    rng = np.random.default_rng(0); worst = 0.0
    for _ in range(50):
        q = np.array([rng.uniform(-5, 5), rng.uniform(-5, 5), rng.uniform(-3, 3)])
        u = np.array([rng.uniform(0, 0.3), rng.uniform(-0.2, 0.2)])
        p = q[:2] + rng.uniform(2, 8, 2) * rng.choice([-1, 1], 2)
        worst = max(worst, np.abs(Fx(q, u) - numjac(lambda x: motion(x, u), q)).max(),
                    np.abs(Fv(q, u) - numjac(lambda v: motion(q, v), u)).max(),
                    np.abs(Hx(q, p) - numjac(lambda x: obs(x, p), q)).max())
    assert worst < 1e-5, worst


def test_predict_equals_dead_reckoning():
    e = EKF(np.zeros(3), np.eye(3) * 1e-4, W); q = np.zeros(3)
    for k in range(200):
        dd, dth = 0.1, 0.03 * np.sin(k / 10)
        e.predict(np.array([dd, dth]), np.diag([1e-4, 1e-5])); q = dead_reckon_step(q, dd, dth)
    assert np.abs(e.q - q).max() < 1e-12


def test_bearing_wrap():
    q = np.array([0.0, 0.0, np.pi - 0.01]); p = np.array([-5.0, -0.1])
    nu = obs(q, p) - obs(q + np.array([0, 0, 0.02]), p); nu[1] = wrap(nu[1])
    assert abs(nu[1]) < 0.05


def test_noise_free_tracks_truth():
    q_true = np.zeros(3); e = EKF(q_true, np.eye(3) * 1e-4, W); lm = np.array([[5, 5], [-5, 5], [0, -6.0]])
    for k in range(200):
        u = np.array([0.1, 0.02]); q_true = motion(q_true, u); e.predict(u, np.zeros((2, 2)))
        for p in lm: e.update(obs(q_true, p), p)
    assert np.abs(e.q - q_true).max() < 1e-6


def test_update_reduces_uncertainty():
    b = EKF(np.zeros(3), np.diag([1.0, 1.0, 0.1]), W); tr0 = np.trace(b.P)
    b.update(obs(np.zeros(3), np.array([6.0, 2.0])), np.array([6.0, 2.0]))
    ev = np.linalg.eigvalsh(b.P)
    assert np.trace(b.P) < tr0 and ev.min() > 0 and np.allclose(b.P, b.P.T)


def test_gating_and_min_range():
    c = EKF(np.zeros(3), np.eye(3) * 1e-3, W)
    assert c.update(obs(np.zeros(3), np.array([6.0, 2.0])) + np.array([3.0, 0.0]), np.array([6.0, 2.0])) is False
    assert c.update(np.array([0.5, 0.0]), np.array([0.5, 0.0])) is False


def _sim(level, seed, with_lm=True, T=600):
    """Robot đi vòng tròn R=10 m; odometry nhiễu ĐÚNG mô hình noise.py; V tính từ dd ĐO ĐƯỢC."""
    noise = LEVELS[level]; r = np.random.default_rng(seed); rs = np.random.default_rng(seed + 10_000)
    qt = np.array([10.0, 0.0, np.pi / 2]); e = EKF(qt, np.diag([0.02 ** 2, 0.02 ** 2, np.deg2rad(0.2) ** 2]), W)
    L = np.array([[10, 0], [0, 10], [-10, 0], [0, -10], [8, 8], [-8, -8.0]]); ns = []
    for k in range(T):
        dd_t, dth_t = 0.1, 0.1 / 10.0
        qt = dead_reckon_step(qt, dd_t, dth_t)
        dd, dth = measure(noise, dd_t, dth_t, r)
        e.predict(np.array([dd, dth]), np.diag(np.square(sigmas(noise, dd))))
        if with_lm and k % 2 == 0:
            for p in L:
                if np.hypot(*(p - qt[:2])) < 8:
                    e.update(obs(qt, p) + rs.normal(0, [0.1, np.deg2rad(1)]), p)
        ns.append(nees(e.q, qt, e.P))
    return np.array(ns), float(np.hypot(*(e.q[:2] - qt[:2])))


def test_consistency_all_levels():
    for level in ["low", "mid", "high"]:
        res = [_sim(level, s) for s in range(30)]
        allv = np.concatenate([x[0] for x in res])
        assert 2.0 < allv.mean() < 4.5, (level, allv.mean())
        assert np.mean(allv < 7.81) > 0.90, (level, np.mean(allv < 7.81))


def test_ekf_beats_dead_reckoning():
    e = np.mean([_sim("mid", s)[1] for s in range(30)])
    d = np.mean([_sim("mid", s, with_lm=False)[1] for s in range(30)])
    assert e < 0.3 * d, (e, d)


if __name__ == "__main__":
    ok = True
    for name, fn in list(globals().items()):
        if name.startswith("test_") and callable(fn):
            try:
                fn(); print("PASS ", name)
            except AssertionError as ex:
                ok = False; print("FAIL ", name, ex)
    for level in ["low", "mid", "high"]:
        res = [_sim(level, s) for s in range(30)]; allv = np.concatenate([x[0] for x in res])
        dr = [_sim(level, s, with_lm=False) for s in range(30)]; drv = np.concatenate([x[0] for x in dr])
        print(f"  [{level:4s}] NEES EKF: TB {allv.mean():.2f}, trong 95%: {100*np.mean(allv<7.81):.1f}%  |  "
              f"NEES DR: TB {drv.mean():.2f}, trong 95%: {100*np.mean(drv<7.81):.1f}%  |  "
              f"sai số cuối EKF {np.mean([x[1] for x in res]):.2f} m vs DR {np.mean([x[1] for x in dr]):.2f} m")
    print("\nTẤT CẢ PASS" if ok else "\nCÓ LỖI")