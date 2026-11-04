# scripts\show_landmarks.py
import sys
sys.path.insert(0, ".")
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from scripts.run_tracking import get_route
from src.maps import corridor_map
from src.landmarks import corridor_landmarks

m, _ = corridor_map()
path, J, _ = get_route("corridor")
lm, _ = corridor_landmarks("8")
plt.figure(figsize=(7, 7))
plt.imshow(m.occ, origin="lower", extent=[0, 50, 0, 50], cmap="Greys", alpha=0.8)
plt.plot(path[:, 0], path[:, 1], "c-")
plt.plot(lm[:, 0], lm[:, 1], "r^")
for k, (x, y) in enumerate(lm): plt.text(x + 0.4, y + 0.4, str(k))
plt.savefig("results/landmarks_corridor.png", dpi=150)