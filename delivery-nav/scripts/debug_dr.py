import sys
sys.path.insert(0, ".")
import numpy as np
from src.vehicle import Bicycle
from src.noise import LEVELS, measure, dead_reckon_step

car = Bicycle((0, 0, 0.3), L=0.8, psi_max=0.6, dt=0.1)
q_hat = car.q.copy()
rng = np.random.default_rng(0)
for k in range(5):
    car.step(1.0, 0.3)
    inc = car.last_inc
    dd, dth = measure(LEVELS["none"], *inc, rng)
    q_hat = dead_reckon_step(q_hat, dd, dth)
    print(k, "inc", inc, "| meas", (dd, dth), "| true", car.q, "| hat", q_hat)