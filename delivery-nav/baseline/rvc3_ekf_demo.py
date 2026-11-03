import sys
import numpy as np
from math import pi
import matplotlib.pyplot as plt
from roboticstoolbox import *

mode = sys.argv[1].upper()          # "A" = dead reckoning, "B" = landmark
OUT = r"D:\robot_project\delivery-nav\baseline"
DATE = "2026-11-03"

V = np.diag([0.02, np.deg2rad(0.5)]) ** 2
P0 = np.diag([0.05, 0.05, np.deg2rad(0.5)]) ** 2
W = np.diag([0.1, np.deg2rad(1)]) ** 2

if mode == "A":
    map = None
    robot = Bicycle(covar=V, animation="car")
    robot.control = RandomPath(workspace=10, seed=0)
    ekf = EKF(robot=(robot, V), P0=P0)
    tag = "deadreckoning"
else:
    map = LandmarkMap(20, workspace=10)
    robot = Bicycle(covar=V, animation="car")
    robot.control = RandomPath(workspace=map, seed=0)
    sensor = RangeBearingSensor(robot=robot, map=map, covar=W,
                                angle=[-pi/2, pi/2], range=4, seed=0, plot=False)
    ekf = EKF(robot=(robot, V), P0=P0, map=map, sensor=(sensor, W))
    tag = "landmark"

ekf.run(T=20)

plt.figure()
if map is not None:
    map.plot()
robot.plot_xy(color="b")
ekf.plot_xy(color="r")
ekf.plot_ellipse()
plt.savefig(f"{OUT}\\rvc3_ekf_{tag}_xy_{DATE}.png", dpi=150)

t = ekf.get_t()
pn = ekf.get_Pnorm()
plt.figure()
plt.plot(t, pn)
plt.grid(True)
plt.xlabel("t (s)")
plt.ylabel("norm(P)")
plt.savefig(f"{OUT}\\rvc3_ekf_{tag}_pnorm_{DATE}.png", dpi=150)
print("done", tag)