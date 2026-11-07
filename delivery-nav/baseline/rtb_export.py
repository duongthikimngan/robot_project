import sys
from pathlib import Path
from math import pi
import numpy as np
import matplotlib
matplotlib.use("Agg")
from roboticstoolbox import Bicycle, RandomPath, LandmarkMap, RangeBearingSensor, EKF

OUT = Path(r"D:\robot_project\delivery-nav\baseline\rtb_replay")
OUT.mkdir(exist_ok=True)
V = np.diag([0.02, np.deg2rad(0.5)]) ** 2
P0 = np.diag([0.05, 0.05, np.deg2rad(0.5)]) ** 2
W = np.diag([0.1, np.deg2rad(1)]) ** 2


def run(seed):
    np.random.seed(seed)                      # phòng khi Bicycle dùng RNG toàn cục
    lmap = LandmarkMap(20, workspace=10, seed=seed)
    robot = Bicycle(covar=V)
    robot.control = RandomPath(workspace=lmap, seed=seed)
    sensor = RangeBearingSensor(robot=robot, map=lmap, covar=W, angle=[-pi / 2, pi / 2],
                                range=4, seed=seed, plot=False)
    ekf = EKF(robot=(robot, V), P0=P0, map=lmap, sensor=(sensor, W))

    true = []                                  # ghi pose thật sau mỗi bước
    orig = robot.step
    def step(*a, **k):
        odo = orig(*a, **k)
        true.append(np.array(robot.x, float))
        return odo
    robot.step = step

    ekf.run(T=20)
    h = ekf.history
    n = min(len(h), len(true))
    if len(h) != len(true):
        print(f"  cảnh báo seed {seed}: history {len(h)} != true {len(true)}, cắt còn {n}")
    xest = np.array([e[1] for e in h[:n]], float)
    odo = np.array([e[2] for e in h[:n]], float)
    P = np.array([e[3] for e in h[:n]], float)
    lm = np.array([e[7] for e in h[:n]]).astype(int)
    z = np.array([e[8] if e[8] is not None else [np.nan, np.nan] for e in h[:n]], float)
    np.savez(OUT / f"seed_{seed:02d}.npz", true=np.array(true[:n]), xest=xest, odo=odo, P=P,
             lm=lm, z=z, landmarks=np.array(lmap.landmarks, float))
    print(f"seed {seed}: {n} bước, {(lm >= 0).sum()} quan sát")


if __name__ == "__main__":
    n = int(sys.argv[1]) if len(sys.argv) > 1 else 1
    for s in range(n):
        run(s)