import numpy as np
from src.maps import corridor_map
from src.landmarks import corridor_landmarks, CONFIGS
from src.sensor import RangeBearingSensor


def test_landmarks_in_free_space_and_not_too_close():
    m, _ = corridor_map()
    for cfg in CONFIGS:
        lm, _ = corridor_landmarks(cfg)
        assert all(m.is_free(*m.to_grid(x, y)) for x, y in lm), cfg


def _sensor(**kw):
    m, _ = corridor_map()
    lm, rng_ = corridor_landmarks("8")
    return RangeBearingSensor(m, lm, max_range=kw.get("max_range", rng_),
                              sigma_r=kw.get("sr", 0.0), sigma_b=kw.get("sb", 0.0))


def test_noise_free_values_exact():
    s = _sensor()
    q = (3.0, 10.0, 0.0)                       # nhin thang ve +x, landmark 0 o (2, 10)
    got = {i: (r, b) for i, r, b in s.measure(q)}
    assert abs(got[0][0] - 1.0) < 1e-9 and abs(abs(got[0][1]) - np.pi) < 1e-9


def test_range_limit():
    s = _sensor(max_range=4.0)
    ids = [i for i, _, _ in s.measure((3.0, 10.0, 0.0))]
    assert 0 in ids and 5 not in ids           # (19, 37) cach ~31 m


def test_occlusion_by_wall():
    s = _sensor()
    # (2,10) -> (19,19)... diem (17.5,12) bi khoi (6..16, 6..16) chan khi nhin tu (2,10)
    ids = [i for i, _, _ in s.measure((2.0, 10.0, 0.0))]
    assert 1 not in ids


def test_noise_stream_independent_of_visibility():
    m, _ = corridor_map()
    lm, _ = corridor_landmarks("8")
    a = RangeBearingSensor(m, lm, max_range=8.0, seed=3)
    b = RangeBearingSensor(m, lm, max_range=4.0, seed=3)
    q = (3.0, 10.0, 0.0)
    ra = {i: (r, bb) for i, r, bb in a.measure(q)}
    rb = {i: (r, bb) for i, r, bb in b.measure(q)}
    for i in rb:                               # cung landmark -> cung so do (cung nhieu)
        assert abs(ra[i][0] - rb[i][0]) < 1e-12