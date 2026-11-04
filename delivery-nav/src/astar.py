import heapq
import math

SQ2 = math.sqrt(2.0)
MOVES = [(1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0),
         (1, 1, SQ2), (1, -1, SQ2), (-1, 1, SQ2), (-1, -1, SQ2)]


def _h(a, b):
    """Heuristic octile (don vi: o)."""
    dx, dy = abs(a[0] - b[0]), abs(a[1] - b[1])
    return (dx + dy) + (SQ2 - 2.0) * min(dx, dy)


def astar(gmap, start_xy, goal_xy):
    """A* 8 huong tren luoi (nen truyen ban do DA PHONG).
    Tra ve (path, cost_m): path la list (x, y) tam o, cost_m la do dai (m).
    Tra ve (None, None) neu diem dau/cuoi bi chan hoac khong co duong."""
    s = gmap.to_grid(*start_xy)
    g = gmap.to_grid(*goal_xy)
    if not gmap.is_free(*s) or not gmap.is_free(*g):
        return None, None

    openq = [(_h(s, g), 0.0, s)]
    best = {s: 0.0}
    parent = {}
    closed = set()

    while openq:
        _, gc, cur = heapq.heappop(openq)
        if cur in closed:
            continue
        if cur == g:
            cells = [cur]
            while cur in parent:
                cur = parent[cur]
                cells.append(cur)
            cells.reverse()
            return [gmap.to_world(*c) for c in cells], gc * gmap.res
        closed.add(cur)
        for dx, dy, c in MOVES:
            nb = (cur[0] + dx, cur[1] + dy)
            if not gmap.is_free(*nb):
                continue
            if dx and dy and not (gmap.is_free(cur[0] + dx, cur[1]) and
                                  gmap.is_free(cur[0], cur[1] + dy)):
                continue
            ng = gc + c
            if ng < best.get(nb, float("inf")):
                best[nb] = ng
                parent[nb] = cur
                heapq.heappush(openq, (ng + _h(nb, g), ng, nb))
    return None, None