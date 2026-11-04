import sys
sys.path.insert(0, ".")
import numpy as np
from dataclasses import replace
import src.noise as nz
from scripts.run_tracking import run_episode, get_route

MAP = sys.argv[1] if len(sys.argv) > 1 else "corridor"
SCALE = float(sys.argv[2]) if len(sys.argv) > 2 else 1.0   # nhan c_th va k_d
N = 15
path, J, clr = get_route(MAP)
for name in ["low", "mid", "high"]:
    base = nz.LEVELS[name]
    nz.LEVELS[name] = replace(base, c_th=base.c_th * SCALE, k_d=base.k_d * SCALE)
    errs = [run_episode(path, J, 1.5, "geometric", clearance=clr, loc="dr",
                        noise_level=name, seed=s)[0]["loc_err_end"] for s in range(N)]
    print(name, "c_th(deg/sqrt m) =", round(np.rad2deg(nz.LEVELS[name].c_th), 3),
          "| RMS sai so cuoi:", round(float(np.sqrt(np.mean(np.square(errs)))), 2), "m")
    nz.LEVELS[name] = base