"""PID dạng ISA (slide Ch.3 p2) + quy tắc chỉnh S-IMC cho quá trình bậc nhất."""
import numpy as np


class PID:
    """u = kp*(e + (1/Ti)∫e + Td*de/dt), đạo hàm có lọc (hằng số Tds), có chống bão hòa tích phân."""

    def __init__(self, kp, ti, td=0.0, dt=0.1, umin=-np.inf, umax=np.inf, tds=0.1):
        self.kp, self.ti, self.td, self.dt, self.umin, self.umax, self.tds = kp, ti, td, dt, umin, umax, tds
        self.I = self.e_prev = self.d = 0.0

    def __call__(self, sp, mv):
        e = sp - mv
        de = (e - self.e_prev) / self.dt
        self.d += self.dt / (self.tds + self.dt) * (de - self.d)
        self.e_prev = e
        u_unsat = self.kp * (e + self.I / self.ti + self.td * self.d)
        u = float(np.clip(u_unsat, self.umin, self.umax))
        if u == u_unsat:                          # chỉ tích phân khi chưa bão hòa (anti-windup)
            self.I += e * self.dt
        return u


def simc_pi(K, T, tau, Tc):
    """Quy tắc S-IMC cho PI (slide): Kp = T / (K (Tc + tau)),  Ti = min(T, 4 (Tc + tau))."""
    return T / (K * (Tc + tau)), min(T, 4 * (Tc + tau))
