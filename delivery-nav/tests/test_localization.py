import numpy as np
from src.localization import DeadReckoning


def test_matches_rvc3_reference():
    # giá trị tham chiếu lấy từ RVC3: f([0,0,0], [0.1025, 0.02978])
    q = DeadReckoning([0, 0, 0]).update(0.1025, 0.02978)
    assert np.allclose(q, [0.1025, 0.0, 0.02978])


def test_no_drift_without_motion():
    dr = DeadReckoning([1.0, 2.0, 0.5])
    for _ in range(100):
        q = dr.update(0.0, 0.0)
    assert np.allclose(q, [1.0, 2.0, 0.5])


def test_straight_line():
    dr = DeadReckoning([0, 0, 0])
    for _ in range(100):
        q = dr.update(0.1, 0.0)
    assert np.allclose(q, [10.0, 0.0, 0.0])