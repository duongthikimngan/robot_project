"""Kiểm chứng nguyên nhân thất bại của một cấu hình landmark (mặc định lm3_r4, nhiễu high).
  Phần A  (cover): phủ sóng landmark dọc tuyến, quãng 'mù' trước từng điểm giao, sai số dự báo
  Phần B  (fail) : bảng nguyên nhân thất bại trên N seed
  Phần C  (gate) : thí nghiệm cổng - nới/tắt cổng có cứu được các lượt lỗi không

    python scripts/diag_lm3r4.py --map corridor --lm lm3_r4 --noise high --seeds 30 --part all
    python scripts/diag_lm3r4.py --demo --lm lm3 --part all --seeds 8           # thử khung
Giả định: run_tracking.py có get_route, make_sensor, LM_CONFIGS, EKF_GATE, EKF_MIN_RANGE, run_episode
(đã có trong bản tôi đưa; nếu bạn đã đổi tên, sửa ở đây).
"""
import argparse, sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT)); sys.path.insert(0, str(ROOT / "scripts"))
import numpy as np
import pandas as pd
import run_tracking as rt
from src.smooth import line_free
from src.noise import LEVELS

OUT = ROOT / "results" / "giaidoan4_localization"


def resample_path(path, step):
    seg = np.hypot(*np.diff(path, axis=0).T); s = np.r_[0, np.cumsum(seg)]
    ss = np.arange(0, s[-1] + 1e-9, step)
    return np.stack([np.interp(ss, s, path[:, 0]), np.interp(ss, s, path[:, 1])], 1), ss


def part_cover(path, junctions, name, lm, noise, step=0.25):
    sensor = rt.make_sensor(name, lm, 0)
    pts, s = resample_path(path, step)
    usable = np.zeros(len(pts), int)
    for i, p in enumerate(pts):
        for q in sensor.lm:
            d = float(np.hypot(*(q - p)))
            if rt.EKF_MIN_RANGE <= d <= sensor.max_range and line_free(sensor.map, tuple(p), tuple(q)):
                usable[i] += 1
    blind = usable == 0
    # quãng mù liên tục dài nhất
    longest, cur = 0.0, 0.0
    for b in blind:
        cur = cur + step if b else 0.0; longest = max(longest, cur)
    c_th = LEVELS[noise].c_th
    sig = lambda x: c_th * x ** 1.5 / np.sqrt(3)             # sai số ngang 1σ sau quãng đi x (m) không có landmark (noise.py)
    print(f"\n[A] PHỦ SÓNG  {name} / {lm} / nhiễu {noise}: {len(sensor.lm)} landmark, tầm {sensor.max_range:g} m")
    print(f"    tuyến dài {s[-1]:.1f} m | không thấy landmark nào: {100 * blind.mean():.1f}% chiều dài | "
          f"quãng mù liên tục dài nhất: {longest:.1f} m -> sai số ngang 1σ dự báo {sig(longest):.2f} m (R_REACH = {rt.R_REACH:g} m)")
    rows = []
    for j, J in enumerate(junctions, 1):
        k = int(np.argmin(np.hypot(*(pts - J).T)))
        back = 0.0
        i = k
        while i > 0 and blind[i]:
            back += step; i -= 1                          # quãng mù ngay trước điểm giao
        rows.append(dict(junction=j, x=round(float(J[0]), 1), y=round(float(J[1]), 1),
                         n_usable_at=int(usable[k]), blind_before_m=round(back, 1),
                         pred_sigma_m=round(float(sig(back)), 2)))
    df = pd.DataFrame(rows); print(df.to_string(index=False))
    OUT.mkdir(parents=True, exist_ok=True); df.to_csv(OUT / f"diag_cover_{name}_{lm}_{noise}.csv", index=False)


def run_one(path, J, clr, name, lm, noise, seed, gate=None, default_gate=True):
    old = rt.EKF_GATE
    if not default_gate: rt.EKF_GATE = gate
    try:
        sensor = rt.make_sensor(name, lm, seed)
        return rt.run_episode(path, J, 1.5, "geometric", clearance=clr, loc="ekf", noise_level=noise,
                              seed=seed, sensor=sensor)[0]
    finally:
        rt.EKF_GATE = old


def classify(r):
    if r["success"]: return "thành công"
    if r["collisions"] > 0: return "va chạm"
    return "lỡ điểm giao"


def part_fail(path, J, clr, name, lm, noise, seeds):
    rows = [run_one(path, J, clr, name, lm, noise, s) for s in range(seeds)]
    df = pd.DataFrame(rows); df["cause"] = df.apply(classify, axis=1)
    df["mat_nhat_quan"] = df.nees_mean > 4.5
    df["rej_ratio"] = df.n_rej / (df.n_obs + df.n_rej).clip(lower=1)
    print(f"\n[B] NGUYÊN NHÂN THẤT BẠI  {name} / {lm} / nhiễu {noise}, {seeds} seed")
    print(pd.crosstab(df.cause, df.mat_nhat_quan, margins=True).rename(columns={False: "NEES nhất quán", True: "NEES mất nhất quán"}).to_string())
    bad = df[df.cause != "thành công"][["seed", "cause", "n_reached", "collisions", "loc_err_rms", "nees_mean", "n_obs", "n_rej", "rej_ratio"]]
    print("\n    các lượt thất bại:"); print(bad.round(3).to_string(index=False) if len(bad) else "    (không có)")
    OUT.mkdir(parents=True, exist_ok=True); df.to_csv(OUT / f"diag_fail_{name}_{lm}_{noise}.csv", index=False)
    return df


def part_gate(path, J, clr, name, lm, noise, seeds):
    print(f"\n[C] THÍ NGHIỆM CỔNG  {name} / {lm} / nhiễu {noise}, {seeds} seed (min_range giữ nguyên {rt.EKF_MIN_RANGE:g} m)")
    out = []
    for g in (rt.EKF_GATE, 13.82, None):
        rs = pd.DataFrame([run_one(path, J, clr, name, lm, noise, s, gate=g, default_gate=False) for s in range(seeds)])
        if "seed" not in rs.columns:
            rs.insert(0, "seed", range(len(rs)))
        gname = "off" if g is None else format(g, "g")
        rs.to_csv(OUT / f"diag_gate_seeds_{name}_{lm}_{noise}_{gname}.csv", index=False)
        out.append(dict(gate="tắt" if g is None else f"{g:g}", success=round(rs.success.mean(), 3),
                        loc_err_rms=round(rs.loc_err_rms.mean(), 3), loc_err_rms_med=round(rs.loc_err_rms.median(), 3),
                        nees_mean=round(rs.nees_mean.mean(), 2), n_obs=round(rs.n_obs.mean(), 0), n_rej=round(rs.n_rej.mean(), 1)))
    df = pd.DataFrame(out); print(df.to_string(index=False))
    OUT.mkdir(parents=True, exist_ok=True); df.to_csv(OUT / f"diag_gate_{name}_{lm}_{noise}.csv", index=False)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--map", default="corridor"); ap.add_argument("--demo", action="store_true")
    ap.add_argument("--lm", default="lm3_r4"); ap.add_argument("--noise", default="high", choices=list(LEVELS))
    ap.add_argument("--seeds", type=int, default=30)
    ap.add_argument("--part", default="all", choices=["all", "cover", "fail", "gate"])
    a = ap.parse_args()
    name = "demo" if a.demo else a.map
    path, J, clr = rt.demo_route() if a.demo else rt.get_route(a.map)
    if a.part in ("all", "cover"): part_cover(path, J, name, a.lm, a.noise)
    if a.part in ("all", "fail"): part_fail(path, J, clr, name, a.lm, a.noise, a.seeds)
    if a.part in ("all", "gate"): part_gate(path, J, clr, name, a.lm, a.noise, a.seeds)


if __name__ == "__main__":
    main()