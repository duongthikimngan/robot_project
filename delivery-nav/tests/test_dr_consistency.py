import numpy as np
from src.vehicle import Bicycle, wrap
from src.noise import dead_reckon_step


def test_dr_matches_vehicle_without_noise():
    veh = Bicycle(q0=(1.0, 2.0, 0.3), L=1.0, psi_max=0.6, dt=0.1)
    est = veh.q.copy()
    for k in range(500):
        veh.step(1.5, 0.4 * np.sin(0.05 * k))
        dd, dth = veh.last_inc
        est = dead_reckon_step(est, dd, dth)
    assert np.hypot(*(veh.q[:2] - est[:2])) < 1e-9
    assert abs(wrap(veh.q[2] - est[2])) < 1e-9