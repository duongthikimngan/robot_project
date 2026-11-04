import numpy as np
from src.vehicle import wrap
from src.noise import LEVELS, true_increment, measure, dead_reckon_step


def _arc_poses(R=5.0, step=0.05, n=400):
    a = np.arange(n + 1) * step / R
    return np.c_[R * np.sin(a), R * (1 - np.cos(a)), a]


def test_dead_reckoning_exact_without_noise():
    poses = _arc_poses()
    q = poses[0].copy()
    for p0, p1 in zip(poses[:-1], poses[1:]):
        dd, dth = true_increment(p0, p1)
        q = dead_reckon_step(q, dd, dth)
    assert np.allclose(q[:2], poses[-1, :2], atol=1e-9)
    assert abs(wrap(q[2] - poses[-1, 2])) < 1e-9


def test_level_none_changes_nothing():
    rng = np.random.default_rng(0)
    dd, dth = measure(LEVELS["none"], 0.1, 0.02, rng)
    assert dd == 0.1 and dth == 0.02


def test_heading_drift_matches_formula():
    noise = LEVELS["mid"]
    rng = np.random.default_rng(1)
    s_len, step, n_runs = 100.0, 0.1, 300
    n = int(s_len / step)
    err = [sum(measure(noise, step, 0.0, rng)[1] for _ in range(n)) for _ in range(n_runs)]
    expected = noise.c_th * np.sqrt(s_len)
    assert abs(np.std(err) / expected - 1) < 0.15


if __name__ == "__main__":
    test_dead_reckoning_exact_without_noise()
    test_level_none_changes_nothing()
    test_heading_drift_matches_formula()
    print("ok")