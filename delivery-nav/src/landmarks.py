import numpy as np

CORRIDOR_ALL = np.array([
    [2.0, 10.0], [17.5, 12.0], [27.0, 17.0], [33.0, 25.0],
    [30.0, 33.5], [19.0, 37.0], [10.0, 33.5], [2.0, 30.0],
])

CONFIGS = {
    "3":    dict(idx=[0, 3, 6],                  max_range=8.0),
    "8":    dict(idx=list(range(8)),             max_range=8.0),
    "8lim": dict(idx=list(range(8)),             max_range=4.0),
}


def corridor_landmarks(config):
    c = CONFIGS[config]
    return CORRIDOR_ALL[c["idx"]].copy(), c["max_range"]