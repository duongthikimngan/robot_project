"""Pure Pursuit (slide Ch.3 §3.1.5). Hai luật lái:
   law='geometric': psi = atan(2 L sin(alpha) / d)     (Pure Pursuit kinh điển)
   law='slide'    : psi = Kh * (theta* ⊖ theta)         (slide §3.1.3, dùng trong sl_pursuit)

Look-ahead:
   - cố định: d truyền vào __init__ (hoặc truyền d vào __call__)
   - thích nghi theo độ cong (adaptive=dict(d_min, d_max, c, look, tau_up)):
        d_raw = clip(c / kappa_max, d_min, d_max), kappa_max = độ cong lớn nhất trong `look` mét phía trước
        d giảm NGAY khi sắp vào góc, tăng từ từ với hằng số thời gian tau_up (tránh d nhảy)
"""
import numpy as np
from .vehicle import wrap

SPACING = 0.05


def densify(path, spacing=SPACING):
    """Nội suy lại đường về bước nhỏ (đường 0,5 m quá thưa để chọn điểm mục tiêu mượt)."""
    path = np.asarray(path, float)
    seg = np.hypot(*np.diff(path, axis=0).T)
    s = np.r_[0, np.cumsum(seg)]
    ss = np.r_[np.arange(0, s[-1], spacing), s[-1]]
    return np.stack([np.interp(ss, s, path[:, 0]), np.interp(ss, s, path[:, 1])], 1), ss


def curvature(ref, h=0.5, spacing=SPACING):
    """Độ cong Menger (1/m) tại mỗi điểm, từ 3 điểm cách nhau h mét (không nhạy với nhiễu số)."""
    k = int(round(h / spacing))
    kap = np.zeros(len(ref))
    if len(ref) <= 2 * k:
        return kap
    a, b, c = ref[:-2 * k], ref[k:-k], ref[2 * k:]
    ab, bc, ac = b - a, c - b, c - a
    cross = ab[:, 0] * bc[:, 1] - ab[:, 1] * bc[:, 0]
    den = np.hypot(*ab.T) * np.hypot(*bc.T) * np.hypot(*ac.T)
    kap[k:-k] = 2 * np.abs(cross) / np.maximum(den, 1e-9)
    return kap


class PurePursuit:
    def __init__(self, path, L, psi_max, d=3.0, law="geometric", kh=1.5, window_m=20.0,
                 adaptive=None, dt=0.1):
        self.ref, self.s = densify(path)
        self.L, self.psi_max, self.d, self.law, self.kh = L, psi_max, d, law, kh
        self.window = int(window_m / SPACING)
        self.j = 0
        self.ad, self.dt = adaptive, dt
        self.d_f = None          # d đã lọc (chế độ thích nghi)
        self.d_cur = d           # d đang dùng ở bước gần nhất (để ghi log)
        if adaptive:
            self.kap = curvature(self.ref)

    def nearest(self, pos):
        hi = min(self.j + self.window, len(self.ref))
        self.j += int(np.argmin(np.hypot(*(self.ref[self.j:hi] - pos).T)))
        return self.j

    def target(self, d):
        s_t = min(self.s[self.j] + d, self.s[-1])
        return np.array([np.interp(s_t, self.s, self.ref[:, 0]), np.interp(s_t, self.s, self.ref[:, 1])])

    def adaptive_d(self):
        a = self.ad
        i1 = int(np.searchsorted(self.s, self.s[self.j] + a["look"]))
        kmax = float(self.kap[self.j:max(i1, self.j + 1)].max())
        d_raw = float(np.clip(a["c"] / max(kmax, 1e-3), a["d_min"], a["d_max"]))
        if self.d_f is None or d_raw < self.d_f:
            self.d_f = d_raw                                                    # giảm ngay trước khi vào góc
        else:
            self.d_f += (1 - np.exp(-self.dt / a["tau_up"])) * (d_raw - self.d_f)   # tăng từ từ
        return self.d_f

    def __call__(self, pose, d=None):
        self.nearest(pose[:2])                       # nearest chạy trước để có chỉ số j
        if d is None:
            d = self.adaptive_d() if self.ad else self.d
        self.d_cur = d
        tgt = self.target(d)
        alpha = wrap(np.arctan2(tgt[1] - pose[1], tgt[0] - pose[0]) - pose[2])
        if self.law == "geometric":
            ld = max(np.hypot(*(tgt - pose[:2])), 1e-6)          # khoảng cách thật tới điểm mục tiêu
            psi = np.arctan2(2 * self.L * np.sin(alpha), ld)
        else:
            psi = self.kh * alpha
        return float(np.clip(psi, -self.psi_max, self.psi_max)), alpha, tgt

    def cross_track(self, pos):
        """Sai số ngang: khoảng cách từ vị trí THẬT tới điểm gần nhất trên đường."""
        return float(np.min(np.hypot(*(self.ref - pos).T)))

    def finished(self, pos, tol=0.3):
        return self.j >= len(self.ref) - 5 and np.hypot(*(self.ref[-1] - pos)) < tol