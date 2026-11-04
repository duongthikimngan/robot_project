import numpy as np
from src.vehicle import wrap
from src.smooth import line_free


class RangeBearingSensor:
    """Do (range, bearing) toi cac landmark da biet. 360 do, co che khuat boi ban do THAT.
    Moi lan goi sinh nhieu cho TAT CA landmark (thay hay khong) de chuoi ngau nhien co dinh."""

    def __init__(self, gmap, landmarks, max_range=8.0, sigma_r=0.1,
                 sigma_b=np.deg2rad(1.0), seed=0):
        self.map, self.lm = gmap, np.asarray(landmarks, float)
        self.max_range, self.sr, self.sb = max_range, sigma_r, sigma_b
        self.rng = np.random.default_rng([seed, 1])     # luong RIENG, khong dung chung voi odometry

    def R(self):
        return np.diag([self.sr ** 2, self.sb ** 2])

    def measure(self, q_true):
        """Tra ve list (i, r, b) cho cac landmark thay duoc."""
        noise = self.rng.standard_normal((len(self.lm), 2))
        out = []
        for i, (lx, ly) in enumerate(self.lm):
            dx, dy = lx - q_true[0], ly - q_true[1]
            r = float(np.hypot(dx, dy))
            if r > self.max_range or not line_free(self.map, (q_true[0], q_true[1]), (lx, ly)):
                continue
            b = float(wrap(np.arctan2(dy, dx) - q_true[2]))
            out.append((i, r + self.sr * noise[i, 0], float(wrap(b + self.sb * noise[i, 1]))))
        return out