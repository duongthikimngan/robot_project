"""EKF định vị (x, y, theta): dự đoán bằng odometry, hiệu chỉnh bằng landmark range-bearing.
Slide Ch.4 §4.4 (EKF), §4.5.2 (odometry), §4.5.3 (landmark).

Mô hình chuyển động DÙNG CHÍNH `dead_reckon_step` của noise.py -> chỉ có MỘT công thức dead reckoning:
dead reckoning = EKF chỉ gọi predict(); EKF đầy đủ = predict() + update().
"""
import numpy as np
from .noise import dead_reckon_step
from .vehicle import wrap


def motion(q, u):
    """q+ = f(q, u),  u = (dd, dth). Cùng công thức với dead_reckon_step (hướng trung điểm)."""
    return dead_reckon_step(q, u[0], u[1])


def Fx(q, u):
    """Jacobian df/dq."""
    mid = q[2] + 0.5 * u[1]
    return np.array([[1, 0, -u[0] * np.sin(mid)],
                     [0, 1,  u[0] * np.cos(mid)],
                     [0, 0, 1]])


def Fv(q, u):
    """Jacobian df/du với u = (dd, dth): ảnh hưởng của nhiễu đo odometry."""
    mid = q[2] + 0.5 * u[1]
    return np.array([[np.cos(mid), -0.5 * u[0] * np.sin(mid)],
                     [np.sin(mid),  0.5 * u[0] * np.cos(mid)],
                     [0.0, 1.0]])


def obs(q, p):
    """z = h(q, p) = (range, bearing) tới landmark p."""
    dx, dy = p[0] - q[0], p[1] - q[1]
    return np.array([np.hypot(dx, dy), wrap(np.arctan2(dy, dx) - q[2])])


def Hx(q, p):
    """Jacobian dh/dq."""
    dx, dy = p[0] - q[0], p[1] - q[1]
    r2 = dx * dx + dy * dy
    r = np.sqrt(r2)
    return np.array([[-dx / r, -dy / r, 0.0],
                     [dy / r2, -dx / r2, -1.0]])


class EKF:
    def __init__(self, q0, P0, W):
        self.q, self.P, self.W = np.array(q0, float), np.array(P0, float), np.asarray(W, float)
        self.n_used = self.n_rejected = 0

    def predict(self, u, V):
        """V = hiệp phương sai nhiễu odometry CỦA BƯỚC NÀY (2x2) = diag(sigma_d², sigma_th²),
        tính từ noise.sigmas(noise, dd) với đúng dd của bước đó."""
        F, G = Fx(self.q, u), Fv(self.q, u)
        self.q = motion(self.q, u)
        self.P = F @ self.P @ F.T + G @ np.asarray(V, float) @ G.T

    def update(self, z, p, gate=9.21, min_range=1.0):
        """Hiệu chỉnh theo 1 landmark đã biết vị trí p (biết đúng ID). Trả về True nếu được dùng.
        - min_range: bỏ landmark quá gần (tuyến tính hóa kém, có thể làm EKF phân kỳ)
        - gate: ngưỡng Mahalanobis² (9.21 = chi² 99%, 2 bậc tự do), loại quan sát bất thường"""
        if np.hypot(p[0] - self.q[0], p[1] - self.q[1]) < min_range:
            return False
        H = Hx(self.q, p)
        nu = z - obs(self.q, p)
        nu[1] = wrap(nu[1])                                   # chuẩn hóa góc của phần dư
        S = H @ self.P @ H.T + self.W
        if gate is not None and nu @ np.linalg.solve(S, nu) > gate:
            self.n_rejected += 1
            return False
        K = self.P @ H.T @ np.linalg.inv(S)
        self.q = self.q + K @ nu
        self.q[2] = wrap(self.q[2])
        A = np.eye(3) - K @ H
        self.P = A @ self.P @ A.T + K @ self.W @ K.T          # dạng Joseph: P luôn đối xứng, xác định dương
        self.n_used += 1
        return True


def nees(q_est, q_true, P):
    """NEES 3 bậc tự do; kỳ vọng ~3 nếu bộ lọc nhất quán (góc đã chuẩn hóa)."""
    e = np.array(q_est, float) - np.asarray(q_true, float)
    e[2] = wrap(e[2])
    return float(e @ np.linalg.solve(P, e))