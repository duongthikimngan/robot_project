import sys
sys.path.insert(0, ".")
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from scripts.run_tracking import run_episode, get_route

path, J, clr = get_route("corridor")
out, traj, log = run_episode(path, J, 1.5, "geometric", clearance=clr,
                             loc="dr", noise_level="mid", seed=0)
print(out)
hat = log[:, 5:7]
plt.figure(figsize=(7, 7))
plt.plot(path[:, 0], path[:, 1], "c-", label="duong tham chieu")
plt.plot(traj[:, 0], traj[:, 1], "b-", label="quy dao that")
plt.plot(hat[:, 0], hat[:, 1], "r--", label="uoc luong (DR)")
plt.plot(J[:, 0], J[:, 1], "go")
plt.axis("equal"); plt.legend(); plt.grid(True)
plt.savefig("results/phaseB_dr_mid_seed0.png", dpi=150)