import numpy as np
from src.vehicle import wrap


class DeadReckoning:
    """Tích phân odometry đo. Cập nhật theo RVC3 (Bicycle.f):
    x' = x + d cos(th), y' = y + d sin(th), th' = th + dth."""

    def __init__(self, q0):
        self.q = np.array(q0, dtype=float)

    def update(self, d_meas, dth_meas):
        x, y, th = self.q
        self.q = np.array([x + d_meas * np.cos(th),
                           y + d_meas * np.sin(th),
                           wrap(th + dth_meas)])
        return self.q.copy()