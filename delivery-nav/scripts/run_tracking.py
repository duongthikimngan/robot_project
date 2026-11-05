"""Giai đoạn 3: chạy Pure Pursuit (d cố định hoặc look-ahead thích nghi theo độ cong)
trên tuyến của Giai đoạn 2. Dùng POSE THẬT (chưa có nhiễu/định vị - phần đó là Giai đoạn 4).

    python scripts/run_tracking.py --demo
    python scripts/run_tracking.py --map corridor --d 0.5 1 1.5 2 3 4 5
    python scripts/run_tracking.py --map open --law slide
    python scripts/run_tracking.py --map corridor --adaptive                    # dùng ADAPT mặc định
    python scripts/run_tracking.py --map corridor --ad 1.5 3.0 0.8 4 1.0        # d_min d_max c look tau_up

Yêu cầu: src/pursuit.py đã có `adaptive`, `dt`, `d_cur`.
"""
import argparse, csv, sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from src.vehicle import Bicycle
from src.pursuit import PurePursuit
from src.pid import PID, simc_pi

from scipy.ndimage import distance_transform_edt
from src import maps
from src.astar import astar
from src.smooth import shortcut, resample
from src.noise import LEVELS, measure, sigmas               # <<< GD4
from src.ekf import EKF, nees                                # <<< GD4
from src.sensor import RangeBearingSensor                    # <<< GD4

# ---- tham số xe và mô phỏng (cố định cho mọi thí nghiệm) ----
L, PSI_MAX, DT, V_MAX = 0.8, 0.6, 0.1, 1.0
R_PLAN = 1.0      # bán kính phồng lúc lập đường, trùng R_ROBOT trong run_pipeline.py
R_BODY = 0.4      # bán kính thân xe (m)
R_MIN = L / np.tan(PSI_MAX)                 # bán kính quay tối thiểu trục sau (~1.17 m)
R_FLOOR_90 = R_MIN * (np.sqrt(2) - 1)       # sàn lý thuyết cắt góc 90° (~0.48 m)
R_REACH = 1.0                               # bán kính coi là "đã đến điểm giao" (pose thật)
K_V, T_V, TAU_V, TC_V = 1.0, 0.5, 0.1, 0.3  # quá trình vận tốc bậc nhất + trễ; Tc chọn cho S-IMC

# tham số look-ahead thích nghi: d_min, d_max, c, look, tau_up
ADAPT = dict(d_min=1.5, d_max=3.0, c=0.8, look=4.0, tau_up=1.0)

# ---- tham số EKF (Giai đoạn 4) ----
P0 = np.diag([0.02 ** 2, 0.02 ** 2, np.deg2rad(0.2) ** 2])   # xe xuất phát ở pose biết gần như chính xác
SENSOR_EVERY = 2          # cảm biến chạy mỗi 2 bước (5 Hz)
EKF_GATE = 9.21           # chi² 99%, 2 bậc tự do
EKF_MIN_RANGE = 1.0       # m; bỏ landmark quá gần (tuyến tính hóa kém)
LM_CONFIGS = {"lm3": (3, 8.0), "lm8": (8, 8.0), "lm8_r4": (8, 4.0)}   # (số landmark, tầm nhìn tối đa)
DEMO_LM = np.array([[5, -3], [13, 5], [5, 13], [-3, 5], [5, 5], [12, 12], [-2, -2], [8, 2.0]])


def get_route(map_name):
    m, J = getattr(maps, f"{map_name}_map")()
    inf = m.inflate(R_PLAN)
    pts = []
    for a, b in zip(J[:-1], J[1:]):
        p, _ = astar(inf, a, b)
        assert p is not None, f"không có đường {a}->{b}"
        p = [tuple(a)] + p[1:-1] + [tuple(b)]
        r = list(resample(shortcut(inf, p), 0.5))
        pts.extend(r if not pts else r[1:])          # bỏ điểm trùng ở chỗ nối hai đoạn
    path = np.asarray(pts, float)
    junctions = np.asarray(J[1:], float)             # J[0] là kho xuất phát
    edt = distance_transform_edt(~m.occ) * m.res     # khoảng cách tới vật cản THẬT (không phồng)

    def clearance(p):
        i = int(np.clip(p[1] / m.res, 0, edt.shape[0] - 1))
        j = int(np.clip(p[0] / m.res, 0, edt.shape[1] - 1))
        return float(edt[i, j])
    return path, junctions, clearance


def demo_route():
    junctions = np.array([[10, 0], [10, 10], [0, 10], [0, 0.0]])
    path = np.vstack([[0, 0], junctions])
    return path, junctions, None


def get_landmarks(map_name, n):
    """Tọa độ n landmark (Nx2) của bản đồ, lấy theo CONFIGS trong src/landmarks.py."""
    from src.landmarks import CORRIDOR_ALL, CONFIGS
    if map_name != "corridor":
        raise NotImplementedError(f"chưa có landmark cho bản đồ '{map_name}' (src/landmarks.py chỉ có corridor)")
    key = {3: "3", 8: "8"}[n]                      # lm8 và lm8lim cùng dùng 8 điểm, chỉ khác max_range
    return np.asarray(CORRIDOR_ALL, float)[CONFIGS[key]["idx"]]

def make_sensor(map_name, lm_cfg="lm8", seed=0):
    """Tạo cảm biến MỚI cho mỗi lượt chạy (luồng ngẫu nhiên của cảm biến tách khỏi odometry)."""
    n, rmax = LM_CONFIGS[lm_cfg]
    if map_name == "demo":
        return RangeBearingSensor(None, DEMO_LM[:n], max_range=rmax, seed=seed)
    m, _ = getattr(maps, f"{map_name}_map")()
    return RangeBearingSensor(m, get_landmarks(map_name, n), max_range=rmax, seed=seed)


def run_episode(path, junctions, d, law, kh=1.5, clearance=None, t_max=300.0, adaptive=None,
                loc="none", noise_level="none", seed=0, sensor=None):      # <<< GD4
    """d: look-ahead cố định (bị bỏ qua khi adaptive != None).
    loc: 'none' (điều khiển bằng pose thật) | 'dr' (dead reckoning) | 'ekf' (odometry + landmark).
    sensor: RangeBearingSensor MỚI cho mỗi lượt (bắt buộc khi loc='ekf').
    Trả về (kết quả, quỹ đạo thật, log theo thời gian)."""
    car = Bicycle((path[0, 0], path[0, 1], np.arctan2(*(path[1] - path[0])[::-1])),
                  L=L, psi_max=PSI_MAX, dt=DT)
    pp = PurePursuit(path, L, PSI_MAX, d=d, law=law, kh=kh, adaptive=adaptive, dt=DT)
    kp, ti = simc_pi(K_V, T_V, TAU_V, TC_V)
    pid = PID(kp, ti, 0.0, DT, 0.0, 2.0)
    buf = [0.0] * max(1, int(round(TAU_V / DT)))
    v = 0.0
    miss = np.full(len(junctions), np.inf)
    t_reach = np.full(len(junctions), np.nan)
    cte, clr, psis, vs, ds, traj = [], [], [], [], [], [car.q[:2].copy()]
    assert loc in ("none", "dr", "ekf"), loc
    assert loc != "ekf" or sensor is not None, "loc='ekf' cần truyền sensor=make_sensor(...)"
    noise = LEVELS[noise_level]
    rng = np.random.default_rng(seed)                          # luồng ODOMETRY; cảm biến có luồng riêng bên trong sensor
    q_hat = car.q.copy()                                       # pose robot tin, bắt đầu đúng pose thật
    # dr và ekf dùng CÙNG bộ lọc: dr chỉ predict() (P vẫn lan truyền -> tính được NEES cho dead reckoning)
    ekf = EKF(q_hat, P0, sensor.R() if sensor is not None else np.diag([0.1 ** 2, np.deg2rad(1.0) ** 2])) \
        if loc in ("dr", "ekf") else None
    traj_hat, loc_err, sig_l, nees_l = [q_hat[:2].copy()], [], [], []
    th_err = []                                                # <<< MỚI: sai số hướng (ước lượng trừ thật), đã chuẩn hóa góc
    plen = float(np.sum(np.hypot(*np.diff(path, axis=0).T)))
    t_lim = min(t_max, 2.0 * plen / V_MAX)           # chốt chặn nếu xe không về tới cuối tuyến
    for k in range(int(t_lim / DT)):
        q_ctrl = q_hat if loc != "none" else car.q             # điều khiển bằng pose ƯỚC LƯỢNG (không bao giờ dùng pose thật khi loc != none)
        psi, alpha, _ = pp(q_ctrl)
        psis.append(psi); ds.append(pp.d_cur)
        v_ref = V_MAX * float(np.clip(1 - abs(alpha) / 1.0, 0.3, 1.0))
        v_ref = min(v_ref, 0.8 * np.hypot(*(pp.ref[-1] - q_ctrl[:2])) + 0.1)   # giảm tốc ở cuối tuyến
        buf.append(pid(v_ref, v)); u = buf.pop(0)
        v += DT * (K_V * u - v) / T_V                                           # quá trình vận tốc
        car.step(v, psi)
        dd, dth = measure(noise, *car.last_inc, rng)           # gọi ĐÚNG MỘT LẦN mỗi bước, kể cả khi loc="none"
        if ekf is not None:
            ekf.predict(np.array([dd, dth]), np.diag(np.square(sigmas(noise, dd))))   # V đúng bằng nhiễu của noise.py
            if loc == "ekf" and k % SENSOR_EVERY == 0:
                for i, r_, b_ in sensor.measure(car.q):        # pose THẬT chỉ để sinh số đo (mô phỏng cảm biến)
                    ekf.update(np.array([r_, b_]), sensor.lm[i], gate=EKF_GATE, min_range=EKF_MIN_RANGE)
            q_hat = ekf.q.copy()
        else:
            q_hat = car.q.copy()                               # không định vị: ước lượng = pose thật
        vs.append(v)
        pos = car.q[:2]                              # pose THẬT: chỉ dùng để đánh giá
        traj.append(pos.copy()); cte.append(pp.cross_track(pos))
        traj_hat.append(q_hat[:2].copy()); loc_err.append(float(np.hypot(*(q_hat[:2] - pos))))
        th_err.append(float(np.arctan2(np.sin(q_hat[2] - car.q[2]), np.cos(q_hat[2] - car.q[2]))))   # <<< MỚI
        if ekf is not None:
            sig_l.append(float(np.sqrt(ekf.P[0, 0] + ekf.P[1, 1]))); nees_l.append(nees(ekf.q, car.q, ekf.P))
        else:
            sig_l.append(0.0); nees_l.append(0.0)
        dist = np.hypot(*(junctions - pos).T)
        miss = np.minimum(miss, dist)
        new = np.isnan(t_reach) & (dist < R_REACH)
        t_reach[new] = (k + 1) * DT
        if clearance is not None: clr.append(float(clearance(pos)))
        q_stop = q_hat if loc != "none" else car.q
        if pp.j >= len(pp.ref) - 5 and np.hypot(*(pp.ref[-1] - q_stop[:2])) < 0.3:
            break
    cte = np.array(cte)
    coll = int(sum(c < R_BODY for c in clr)) if clr else 0
    done = not np.isnan(t_reach).any()
    t_done = float(np.nanmax(t_reach)) if done else (k + 1) * DT
    nees_a = np.array(nees_l)
    err = np.array(loc_err)                                                                  # <<< MỚI
    t_err1 = float((np.argmax(err > 1.0) + 1) * DT) if (err > 1.0).any() else -1.0           # <<< MỚI: lúc sai số định vị vượt 1 m (-1: không xảy ra)
    out = dict(d=d if adaptive is None else "adaptive", law=law,
               success=int((miss < R_REACH).all() and coll == 0),
               n_reached=int((miss < R_REACH).sum()), n_junctions=len(junctions),
               time=round(t_done, 1),
               cte_rms=round(float(np.sqrt((cte ** 2).mean())), 3), cte_max=round(float(cte.max()), 3),
               t_cte_max=round((int(np.argmax(cte)) + 1) * DT, 1),
               miss_max=round(float(miss.max()), 3), miss_mean=round(float(miss.mean()), 3),
               steer_rate=round(float(np.mean(np.abs(np.diff(psis)))) if len(psis) > 1 else 0.0, 4),
               v_mean=round(float(np.mean(vs)), 3),
               d_mean=round(float(np.mean(ds)), 2), d_min=round(float(np.min(ds)), 2),
               min_clearance=round(min(clr), 3) if clr else "", collisions=coll,
               loc=loc, noise=noise_level, seed=seed,
               loc_err_end=round(loc_err[-1], 3) if loc_err else 0.0,
               loc_err_rms=round(float(np.sqrt(np.mean(np.square(loc_err)))), 3) if loc_err else 0.0,
               nees_mean=round(float(nees_a.mean()), 3) if ekf is not None else 0.0,
               nees_in95=round(float(np.mean(nees_a < 7.81)), 3) if ekf is not None else 0.0,   # chi² 95%, 3 bậc tự do
               sigma_end=round(sig_l[-1], 3) if sig_l else 0.0,
               n_obs=ekf.n_used if ekf is not None else 0, n_rej=ekf.n_rejected if ekf is not None else 0,
               loc_err_max=round(float(err.max()), 3) if len(err) else 0.0,                      # <<< MỚI
               t_err1=t_err1,                                                                    # <<< MỚI
               th_err_max=round(float(np.max(np.abs(th_err))), 3) if th_err else 0.0,            # <<< MỚI
               rej_frac=round(ekf.n_rejected / max(ekf.n_used + ekf.n_rejected, 1), 3) if ekf is not None else 0.0,   # <<< MỚI
               miss_idx=int(np.argmax(miss)),                                                    # <<< MỚI: điểm giao hụt nặng nhất
               missed=" ".join(str(i) for i in np.where(miss >= R_REACH)[0]))                    # <<< MỚI: các điểm giao bị hụt
    # log: t, d, cte, psi, v, x_hat, y_hat, loc_err, sigma_xy, nees, th_err
    log = np.c_[np.arange(1, len(ds) + 1) * DT, ds, cte, psis, vs, np.array(traj_hat)[1:],
                loc_err, sig_l, nees_l, th_err]                                                  # <<< MỚI: thêm cột th_err (cột 10)
    return out, np.array(traj), log
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--map", default="demo")
    ap.add_argument("--demo", action="store_true")
    ap.add_argument("--law", default="geometric", choices=["geometric", "slide"])
    ap.add_argument("--d", type=float, nargs="+", default=[1, 5, 10])
    ap.add_argument("--adaptive", action="store_true", help="look-ahead thích nghi (tham số ADAPT)")
    ap.add_argument("--ad", type=float, nargs=5, default=None,
                    metavar=("DMIN", "DMAX", "C", "LOOK", "TAU"), help="thích nghi với bộ tham số tự chọn")
    ap.add_argument("--rel", type=float, nargs=2, default=None, metavar=("PSI_REL", "CTE_REL"),
                    help="giữ d_min tới khi |psi|<PSI_REL*psi_max và cte<CTE_REL")
    ap.add_argument("--loc", default="none", choices=["none", "dr", "ekf"])  # <<< GD4
    ap.add_argument("--lm", default="lm8", choices=list(LM_CONFIGS), help="cấu hình landmark (khi --loc ekf)")
    ap.add_argument("--noise", default="none", choices=list(LEVELS))          # <<< GD4
    ap.add_argument("--seed", type=int, default=0)                            # <<< GD4
    a = ap.parse_args()
    if a.ad:
        a.adaptive = True
    adapt = dict(zip(["d_min", "d_max", "c", "look", "tau_up"], a.ad)) if a.ad else dict(ADAPT)
    if a.rel:
        a.adaptive = True
        adapt["psi_release"], adapt["cte_release"] = a.rel
    name = "demo" if a.demo else a.map
    loc_tag = "" if a.loc == "none" else f"_{a.loc}_{a.noise}_s{a.seed}"
    suffix = ("_adaptive_" + "_".join(f"{x:g}" for x in a.ad)) if a.ad else ("_adaptive" if a.adaptive else "")
    if a.rel:
        suffix += "_hold_" + "_".join(f"{x:g}" for x in a.rel)
    suffix += loc_tag
    path, junctions, clearance = demo_route() if a.demo else get_route(a.map)
    plen = float(np.sum(np.hypot(*np.diff(path, axis=0).T)))
    print(f"độ dài tuyến {plen:.1f} m, thời gian lý tưởng ~{plen / V_MAX:.0f} s")
    print(f"R_min = {R_MIN:.2f} m, sàn cắt góc 90° = {R_FLOOR_90:.2f} m, R_REACH = {R_REACH:.2f} m")
    if a.adaptive:
        print("adaptive:", adapt)
    out_dir = ROOT / "results" / "giaidoan3_tracking"; out_dir.mkdir(parents=True, exist_ok=True)

    cases = ["adaptive"] if a.adaptive else list(a.d)
    rows, trajs = [], {}
    for c in cases:
        sensor = make_sensor(name, a.lm, a.seed) if a.loc == "ekf" else None      # cảm biến MỚI cho mỗi lượt
        r, tj, log = run_episode(path, junctions, 0.0 if a.adaptive else c, a.law, clearance=clearance,
                                 adaptive=adapt if a.adaptive else None,
                                 loc=a.loc, noise_level=a.noise, seed=a.seed, sensor=sensor)   # <<< trước đây chưa truyền
        rows.append(r); trajs[c] = tj
        print(r)
        if a.adaptive:
            np.savetxt(out_dir / f"log_{name}_{a.law}{suffix}.csv", log, delimiter=",",
                       header="t,d,cte,psi,v,x_hat,y_hat,loc_err,sigma_xy,nees", comments="")
    with open(out_dir / f"tracking_{name}_{a.law}{suffix}.csv", "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys())); w.writeheader(); w.writerows(rows)

    fig, axs = plt.subplots(1, len(cases), figsize=(5 * len(cases), 5), squeeze=False)
    for ax, c in zip(axs[0], cases):
        ax.plot(*path.T, "k--", lw=1, label="tuyến tham chiếu")
        ax.plot(*trajs[c].T, "b", lw=1.5, label="quỹ đạo xe")
        ax.plot(*junctions.T, "m*", ms=12, label="điểm giao")
        ax.set_aspect("equal"); ax.grid(alpha=.3)
        ax.set_title(f"d = {c if isinstance(c, str) else format(c, 'g')}  ({a.law})")
    axs[0][0].legend(fontsize=8); plt.tight_layout()
    plt.savefig(out_dir / f"tracking_{name}_{a.law}{suffix}.png", dpi=120)
    print("Đã lưu vào", out_dir)


if __name__ == "__main__":
    main()