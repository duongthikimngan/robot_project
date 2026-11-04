"""Mô hình bicycle động học (slide Ch.3 §3.1.1) có giới hạn góc lái."""
import numpy as np


def wrap(a):
    return (a + np.pi) % (2 * np.pi) - np.pi


class Bicycle:
    """q = (x, y, theta). Gốc thân xe ở tâm trục sau. Cập nhật theo hướng trung điểm
    nên lái ψ không đổi cho vòng tròn bán kính đúng L/tan(ψ)."""

    def __init__(self, q0=(0.0, 0.0, 0.0), L=1.0, psi_max=0.6, dt=0.1):
        self.q = np.array(q0, float)
        self.L, self.psi_max, self.dt = L, psi_max, dt

    def step(self, v, psi):
        psi = float(np.clip(psi, -self.psi_max, self.psi_max))
        x, y, th = self.q
        dth = v / self.L * np.tan(psi) * self.dt
        self.q = np.array([x + v * self.dt * np.cos(th + dth / 2),
                           y + v * self.dt * np.sin(th + dth / 2),
                           wrap(th + dth)])
        self.last_inc = (v * self.dt, dth)
        return self.q.copy(), psi
