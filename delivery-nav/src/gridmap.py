import numpy as np
from scipy.ndimage import distance_transform_edt


class GridMap:
    """Luoi chiem dung. occ[iy, ix] = True neu co vat can. Toa do the gioi (m)."""

    def __init__(self, size_m=50.0, res=0.5):
        self.res = res
        self.n = int(round(size_m / res))
        self.occ = np.zeros((self.n, self.n), dtype=bool)

    def add_rect(self, x0, y0, x1, y1):
        i0, i1 = int(x0 / self.res), int(np.ceil(x1 / self.res))
        j0, j1 = int(y0 / self.res), int(np.ceil(y1 / self.res))
        self.occ[max(j0, 0):j1, max(i0, 0):i1] = True

    def add_circle(self, cx, cy, r):
        xs = (np.arange(self.n) + 0.5) * self.res
        X, Y = np.meshgrid(xs, xs)
        self.occ |= (X - cx) ** 2 + (Y - cy) ** 2 <= r ** 2

    def add_ellipse(self, cx, cy, a, b, angle=0.0):
        xs = (np.arange(self.n) + 0.5) * self.res
        X, Y = np.meshgrid(xs, xs)
        c, s = np.cos(angle), np.sin(angle)
        u = (X - cx) * c + (Y - cy) * s
        v = -(X - cx) * s + (Y - cy) * c
        self.occ |= (u / a) ** 2 + (v / b) ** 2 <= 1.0

    def add_border(self, thickness=1.0):
        t = int(round(thickness / self.res))
        self.occ[:t, :] = self.occ[-t:, :] = True
        self.occ[:, :t] = self.occ[:, -t:] = True

    def inflate(self, radius):
        """Tra ve luoi moi: o nao cach vat can <= radius (m) bi coi la vat can."""
        dist = distance_transform_edt(~self.occ) * self.res
        out = GridMap.__new__(GridMap)
        out.res, out.n = self.res, self.n
        out.occ = dist <= radius
        return out

    def to_grid(self, x, y):
        return int(x / self.res), int(y / self.res)

    def to_world(self, ix, iy):
        return (ix + 0.5) * self.res, (iy + 0.5) * self.res

    def is_free(self, ix, iy):
        return 0 <= ix < self.n and 0 <= iy < self.n and not self.occ[iy, ix]