import sys; sys.path.insert(0, "scripts")
import run_tracking as rt
path, J, clr = rt.get_route("corridor")
for gate in (rt.EKF_GATE, 1e9):
    rt.EKF_GATE = gate
    out, tj, log = rt.run_episode(path, J, 1.5, "geometric", clearance=clr, loc="ekf", noise_level="high",
                                  seed=7, sensor=rt.make_sensor("corridor", "lm8_r4", 7))
    print(f"gate={gate:g}", {k: out[k] for k in ("success", "n_reached", "collisions", "loc_err_max",
                                                  "loc_err_end", "n_obs", "n_rej", "nees_mean")})