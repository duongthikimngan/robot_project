import numpy as np


def line_free(gmap, p, q, step=None):
    """Doan p->q khong cat o bi chiem (lay mau moi nua o)."""
    step = step or gmap.res * 0.5
    d = np.hypot(q[0] - p[0], q[1] - p[1])
    n = max(int(np.ceil(d / step)), 1)
    for t in np.linspace(0.0, 1.0, n + 1):
        x = p[0] + t * (q[0] - p[0])
        y = p[1] + t * (q[1] - p[1])
        if not gmap.is_free(*gmap.to_grid(x, y)):
            return False
    return True


def shortcut(gmap, path):
    """Rut gon bang line-of-sight (tham lam)."""
    out, i = [path[0]], 0
    while i < len(path) - 1:
        j = len(path) - 1
        while j > i + 1 and not line_free(gmap, path[i], path[j]):
            j -= 1
        out.append(path[j])
        i = j
    return out


def resample(path, ds=0.5):
    """Noi suy deu theo khoang cach ds (m), giu diem dau va cuoi."""
    P = np.array(path, dtype=float)
    seg = np.hypot(*np.diff(P, axis=0).T)
    s = np.r_[0.0, np.cumsum(seg)]
    n = max(int(np.ceil(s[-1] / ds)), 1)
    si = np.linspace(0.0, s[-1], n + 1)
    return np.c_[np.interp(si, s, P[:, 0]), np.interp(si, s, P[:, 1])]