import sys
sys.path.insert(0, ".")
from scripts.run_tracking import run_episode, get_route

path, J, clr = get_route("corridor")
for loc, nz in [("dr", "none"), ("dr", "low"), ("dr", "mid"), ("dr", "high")]:
    out, traj, log = run_episode(path, J, 1.5, "geometric", clearance=clr, loc=loc, noise_level=nz, seed=0)
    print(nz, "| success", out["success"], "| miss_max", out["miss_max"],
          "| loc_err_end", out["loc_err_end"], "| cte_rms", out["cte_rms"])