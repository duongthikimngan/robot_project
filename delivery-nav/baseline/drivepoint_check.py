import numpy as np
import matplotlib.pyplot as plt

PSI_MAX = 0.45 * np.pi  # gia dinh, chinh theo mo hinh that cua nhom

def wrap(a):
    return (a + np.pi) % (2 * np.pi) - np.pi

def run(Kv, Kh, L=1.0, dt=0.01, tmax=15.0):
    xg = np.array([5.0, 5.0])
    x = np.array([8.0, 5.0, np.pi / 2])
    log, t = [], 0.0
    while t < tmax:
        d = xg - x[:2]
        rho = np.hypot(d[0], d[1])
        if rho < 0.1:
            break
        v = Kv * rho
        psi_cmd = Kh * wrap(np.arctan2(d[1], d[0]) - x[2])
        psi = np.clip(psi_cmd, -PSI_MAX, PSI_MAX)
        log.append((t, x[0], x[1], v, psi_cmd, psi))
        x = x + dt * np.array([v * np.cos(x[2]), v * np.sin(x[2]), v / L * np.tan(psi)])
        t += dt
    return np.array(log)

fig, ax = plt.subplots(1, 4, figsize=(18, 4))
for Kv, Kh in [(0.5, 2), (0.5, 4), (4, 2)]:
    r = run(Kv, Kh)
    lab = f"Kv={Kv}, Kh={Kh}"
    ax[0].plot(r[:, 1], r[:, 2], label=lab)
    ax[1].plot(r[:, 0], r[:, 3], label=lab)
    ax[2].plot(r[:, 0], r[:, 4], label=lab)
    ax[3].plot(r[:, 0], r[:, 5], label=lab)
    print(lab, "| t den dich:", round(r[-1, 0], 2), "s | v dau:", round(r[0, 3], 2),
          "| psi_cmd dau:", round(r[0, 4], 2), "| psi_thuc dau:", round(r[0, 5], 2))
for a, ti in zip(ax, ["quy dao xy", "van toc v", "lenh lai (chua gioi han)", "goc lai thuc (gioi han)"]):
    a.set_title(ti); a.grid(True); a.legend()
plt.tight_layout()
plt.savefig("mine_drivepoint_Kv-Kh-compare_2026-11-03.png", dpi=150)
plt.show()