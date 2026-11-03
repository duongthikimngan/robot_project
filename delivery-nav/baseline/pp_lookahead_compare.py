import sys, math
sys.path.insert(0, r"D:\robot_project\external\PythonRobotics\PathTracking\pure_pursuit")
import numpy as np
import matplotlib.pyplot as plt
import pure_pursuit as pp

def run(k, Lfc):
    pp.k, pp.Lfc = k, Lfc
    cx = np.arange(0, 50, 0.5)
    cy = [math.sin(ix / 5.0) * ix / 2.0 for ix in cx]
    state = pp.State(x=0.0, y=-3.0, yaw=0.0, v=0.0)
    course = pp.TargetCourse(cx, cy)
    ind, _ = course.search_target_index(state)
    xs, ys, t, status = [state.x], [state.y], 0.0, "den cuoi duong"
    while len(cx) - 1 > ind:
        if t >= 100.0:
            status = "het 100 s"
            break
        try:
            a = pp.proportional_control(10.0 / 3.6, state.v)
            di, ind = pp.pure_pursuit_steer_control(state, course, ind)
        except IndexError:
            status = "loi chi so (xe lac/chay qua cuoi)"
            break
        state.update(a, di)
        t += pp.dt
        xs.append(state.x); ys.append(state.y)
        if abs(state.x) > 200 or abs(state.y) > 200:
            status = "phan ky (xe bay xa)"
            break
    return np.array(cx), np.array(cy), np.array(xs), np.array(ys), t, status

cases = [("mac dinh k=0.1, Lfc=2", 0.1, 2.0), ("d=1", 0.0, 1.0),
         ("d=5", 0.0, 5.0), ("d=10", 0.0, 10.0)]
fig, axs = plt.subplots(2, 2, figsize=(12, 8))
for ax, (name, k, Lfc) in zip(axs.ravel(), cases):
    cx, cy, xs, ys, t, status = run(k, Lfc)
    err = [np.min(np.hypot(cx - x, cy - y)) for x, y in zip(xs, ys)]
    ax.plot(cx, cy, "r--", label="course")
    ax.plot(xs, ys, "b", label="trajectory")
    ax.set_title(f"{name} | TB={np.mean(err):.2f} m, max={np.max(err):.2f} m\n{status}", fontsize=9)
    ax.grid(True); ax.legend()
    print(name, "| t:", round(t, 1), "s | TB:", round(np.mean(err), 2),
          "| max:", round(np.max(err), 2), "|", status)
plt.tight_layout()
plt.savefig("pyrobotics_purepursuit_d-compare_2026-11-03.png", dpi=150)
plt.show()