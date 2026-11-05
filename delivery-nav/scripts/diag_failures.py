import sys; sys.path.insert(0, "scripts")
import numpy as np, pandas as pd
import run_tracking as rt

CSV = "results/giaidoan4_localization/ekf_check_corridor_lm3-lm8-lm8_r4.csv"
d = pd.read_csv(CSV)
fail = d[(d["loc"] == "ekf") & (d.success == 0)]
path, junc, clr = rt.get_route("corridor")
print("điểm giao:", {i: tuple(np.round(j, 1)) for i, j in enumerate(junc)})

for _, r in fail.iterrows():
    seed, cfg = int(r.seed), r.cfg
    out, tj, log = rt.run_episode(path, junc, 1.5, "geometric", clearance=clr, loc="ekf",
                                  noise_level=r.noise, seed=seed,
                                  sensor=rt.make_sensor("corridor", cfg, seed))
    if abs(out["loc_err_max"] - r.loc_err_max) > 1e-3:
        print("  CẢNH BÁO: không tái lập được lượt này")
    s2 = rt.make_sensor("corridor", cfg, 10_000 + seed)           # cảm biến riêng, chỉ để đếm landmark nhìn thấy
    nvis = lambda p: len(s2.measure(np.array([p[0], p[1], 0.0])))
    err = log[:, 7]
    i1 = int(np.argmax(err > 1.0)) if (err > 1.0).any() else int(np.argmax(err))
    im = int(np.argmax(err))
    vis = np.mean([nvis(tj[k + 1]) for k in range(max(0, i1 - 100), i1 + 1, 10)])   # 10 s trước, mỗi 1 s
    print(f"{cfg:6s} seed {seed:2d} | hụt [{out['missed']}] | err>1m lúc t={log[i1,0]:5.1f}s tại "
          f"({tj[i1+1,0]:5.1f},{tj[i1+1,1]:5.1f}) | LM thấy TB 10 s trước: {vis:.1f} | "
          f"err_max {err[im]:.2f} m lúc t={log[im,0]:.1f}s | rej_frac {out['rej_frac']}")