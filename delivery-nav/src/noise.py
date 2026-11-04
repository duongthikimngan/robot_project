"""Giai đoạn 4: mô hình nhiễu odometry + dead reckoning.

Odometry là gia số theo bước u = (dd, dth): quãng đường và độ đổi hướng giữa hai pose thật liên tiếp.
Nhiễu độc lập theo từng bước:
    dd_đo  = dd  + k_d  * |dd|       * N(0, 1)
    dth_đo = dth + c_th * sqrt(|dd|) * N(0, 1)
Độ lệch chuẩn hướng tích luỹ ~ c_th*sqrt(s); độ lệch chuẩn sai số ngang ~ c_th*s**1.5/sqrt(3).
Xe đứng yên (dd = 0) thì không nhiễu, không trôi.
"""
from dataclasses import dataclass
import numpy as np
from .vehicle import wrap


@dataclass(frozen=True)
class OdomNoise:
    name: str
    k_d: float     # độ lệch chuẩn tương đối của quãng đường (không thứ nguyên)
    c_th: float    # rad / sqrt(m)


LEVELS = {
    "none": OdomNoise("none", 0.0, 0.0),
    "low":  OdomNoise("low",  0.038, np.deg2rad(0.095)),
    "mid":  OdomNoise("mid",  0.095, np.deg2rad(0.285)),
    "high": OdomNoise("high", 0.19,  np.deg2rad(0.76)),
}


def true_increment(q_prev, q_now):
    """Gia số thật (dd, dth) giữa hai pose (x, y, theta)."""
    dd = float(np.hypot(q_now[0] - q_prev[0], q_now[1] - q_prev[1]))
    dth = float(wrap(q_now[2] - q_prev[2]))
    return dd, dth


def sigmas(noise, dd):
    return noise.k_d * abs(dd), noise.c_th * np.sqrt(abs(dd))


def measure(noise, dd, dth, rng):
    """Odometry đo được = gia số thật + nhiễu."""
    s_d, s_th = sigmas(noise, dd)
    return dd + s_d * rng.standard_normal(), dth + s_th * rng.standard_normal()


def dead_reckon_step(q_est, dd, dth):
    """Một bước dead reckoning từ gia số odometry (chính xác với chuyển động cung tròn)."""
    x, y, th = q_est
    mid = th + 0.5 * dth
    return np.array([x + dd * np.cos(mid), y + dd * np.sin(mid), wrap(th + dth)])