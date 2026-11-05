import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import numpy as np
import scripts.run_tracking as rt

path, J, _ = rt.get_route("corridor")
ref = rt.PurePursuit(path, rt.L, rt.PSI_MAX).ref[::5]           # bước 0.25 m
STEP = 0.25

def longest(mask):
    best = cur = 0
    for m in mask:
        cur = cur + 1 if m else 0
        best = max(best, cur)
    return best * STEP

for cfg in ["lm3", "lm8", "lm8_r4"]:
    s = rt.make_sensor("corridor", cfg, seed=0)
    z0 = s.measure(np.array([ref[0, 0], ref[0, 1], 0.0]))
    print(cfg, "mẫu measure():", type(z0), np.shape(z0))            # kiểm tra kiểu dữ liệu trả về
    cnt = np.array([len(s.measure(np.array([x, y, 0.0])) or []) for x, y in ref])
    print(f"{cfg:7s} 0 LM: {np.mean(cnt == 0):4.0%} | <=1 LM: {np.mean(cnt <= 1):4.0%} | "
          f"đoạn không LM dài nhất: {longest(cnt == 0):5.1f} m | số LM TB: {cnt.mean():.1f}")