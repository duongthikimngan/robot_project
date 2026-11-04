"""EKF định vị (x, y, theta): dự đoán bằng odometry, hiệu chỉnh bằng landmark range-bearing.
Slide Ch.4 §4.4 (EKF), §4.5.2 (mô hình odometry), §4.5.3 (landmark).
Dead reckoning = chỉ gọi predict(); EKF = predict() + update()."""
import numpy as np


def wrap(a):
    return (a + np.pi) % (2 * np.pi) - np.pi


def motion(q, u):
    """q+ = f(q, u),  u = (dd, dth) là odometry đo được trong 1 bước."""
    x, y, th = q
    return np.array([x + u[0] * np.cos(th), y + u[0] * np.sin(th), wrap(th + u[1])])


def Fx(q, u):                                   # df/dq
    th = q[2]
    return np.array([[1, 0, -u[0] * np.sin(th)],
                     [0, 1,  u[0] * np.cos(th)],
                     [0, 0, 1]])


def Fv(q, u):                                   # df/du (nhiễu đo odometry)
    th = q[2]
    return np.array([[np.cos(th), 0], [np.sin(th), 0], [0, 1]])


def obs(q, p):
    """z = h(q, p) = (range, bearing) tới landmark p."""
    dx, dy = p[0] - q[0], p[1] - q[1]
    return np.array([np.hypot(dx, dy), wrap(np.arctan2(dy, dx) - q[2])])


def Hx(q, p):                                   # dh/dq
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
        """V = hiệp phương sai nhiễu odometry CỦA BƯỚC NÀY (2x2). Với mô hình nhiễu tăng theo quãng đường,
        V phụ thuộc |dd| nên phải tính lại mỗi bước, và phải dùng đúng công thức của noise.py."""
        F, G = Fx(self.q, u), Fv(self.q, u)
        self.q = motion(self.q, u)
        self.P = F @ self.P @ F.T + G @ np.asarray(V, float) @ G.T

    def update(self, z, p, gate=9.21, min_range=1.0):
        """Cập nhật theo 1 landmark đã biết vị trí p (biết đúng ID). Trả về True nếu được dùng.
        - min_range: bỏ landmark quá gần (tuyến tính hóa kém, dễ làm EKF phân kỳ)
        - gate: ngưỡng Mahalanobis^2 (9.21 = chi2 99%, 2 bậc tự do) loại quan sát bất thường"""
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
    """NEES 3 bậc tự do; kỳ vọng ~3 nếu bộ lọc nhất quán. Chuẩn hóa góc của sai số theta."""
    e = q_est - q_true
    e[2] = wrap(e[2])
    return float(e @ np.linalg.solve(P, e))