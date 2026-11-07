import sys, glob
from pathlib import Path
import numpy as np
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
import src.ekf as E
from src.vehicle import wrap

V = np.diag([0.02, np.deg2rad(0.5)]) ** 2
P0 = np.diag([0.05, 0.05, np.deg2rad(0.5)]) ** 2
W = np.diag([0.1, np.deg2rad(1)]) ** 2


def euler_motion(q, u):
    return np.array([q[0] + u[0] * np.cos(q[2]), q[1] + u[0] * np.sin(q[2]), wrap(q[2] + u[1])])

def euler_Fx(q, u):
    return np.array([[1, 0, -u[0] * np.sin(q[2])], [0, 1, u[0] * np.cos(q[2])], [0, 0, 1]])

def euler_Fv(q, u):
    return np.array([[np.cos(q[2]), 0], [np.sin(q[2]), 0], [0, 1.0]])


def replay(d, euler):
    saved = (E.motion, E.Fx, E.Fv)
    if euler:
        E.motion, E.Fx, E.Fv = euler_motion, euler_Fx, euler_Fv
    try:
        f = E.EKF(np.zeros(3), P0, W)
        n = len(d["odo"])
        Q, PP = np.zeros((n, 3)), np.zeros((n, 3, 3))
        for k in range(n):
            f.predict(d["odo"][k], V)
            if d["lm"][k] >= 0:
                f.update(d["z"][k], d["landmarks"][:, d["lm"][k]], gate=None, min_range=0.0)
            Q[k], PP[k] = f.q, f.P
    finally:
        E.motion, E.Fx, E.Fv = saved
    return Q, PP


def metrics(est, P, true):
    e = est - true
    e[:, 2] = wrap(e[:, 2])
    pos = np.hypot(e[:, 0], e[:, 1])
    sd = np.sqrt(np.stack([P[:, 0, 0], P[:, 1, 1], P[:, 2, 2]], 1))
    in2 = np.mean(np.all(np.abs(e) < 2 * sd, axis=1))
    nees = np.array([e[k] @ np.linalg.solve(P[k], e[k]) for k in range(len(e))])
    return dict(final=pos[-1], rms=float(np.sqrt(np.mean(pos ** 2))), in2s=float(in2),
                nees_med=float(np.median(nees)))


def main():
    files = sorted(glob.glob(str(ROOT / "baseline" / "rtb_replay" / "seed_*.npz")))
    rows = {"RTB": [], "nhóm (trung điểm)": [], "nhóm (Euler, như RTB)": []}
    dx_mid, dx_eul, dP_eul = [], [], []
    for f in files:
        d = np.load(f)
        true, xr, Pr = d["true"], d["xest"], d["P"]
        qm, Pm = replay(d, euler=False)
        qe, Pe = replay(d, euler=True)
        rows["RTB"].append(metrics(xr, Pr, true))
        rows["nhóm (trung điểm)"].append(metrics(qm, Pm, true))
        rows["nhóm (Euler, như RTB)"].append(metrics(qe, Pe, true))
        def dxy(a, b):
            e = a - b; e[:, 2] = wrap(e[:, 2]); return float(np.abs(e).max())
        dx_mid.append(dxy(qm, xr)); dx_eul.append(dxy(qe, xr))
        dP_eul.append(float(np.abs(Pe - Pr).max() / np.abs(Pr).max()))
    print(f"{len(files)} seed\n")
    print("Chênh |x_est nhóm - x_est RTB| lớn nhất theo bước (trung vị trên seed):")
    print(f"  mô hình trung điểm: {np.median(dx_mid):.3e}")
    print(f"  mô hình Euler     : {np.median(dx_eul):.3e}   (kỳ vọng ~1e-9 nếu hai bước cập nhật giống nhau)")
    print(f"  chênh P tương đối (Euler): {np.median(dP_eul):.3e}\n")
    print(f"{'bộ lọc':26s} {'sai số cuối':>12s} {'rms vị trí':>11s} {'trong ±2σ':>10s} {'NEES trung vị':>14s}")
    for name, r in rows.items():
        g = lambda k: np.median([x[k] for x in r])
        print(f"{name:26s} {g('final'):12.3f} {g('rms'):11.3f} {g('in2s'):10.2f} {g('nees_med'):14.2f}")


if __name__ == "__main__":
    main()