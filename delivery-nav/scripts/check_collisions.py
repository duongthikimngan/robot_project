import sys
import pandas as pd

df = pd.read_csv(sys.argv[1])
d = df[df["loc"] == "ekf"] if "loc" in df else df
d = d.assign(collided=(d["collisions"] > 0).astype(int))
print("so episode:", len(d))
print("success mean:", d["success"].mean().round(3))
print("ti le episode co va cham:", d["collided"].mean().round(3))
print("collisions (buoc) trung binh:", d["collisions"].mean().round(1))
print("collisions (buoc) trung vi:", d["collisions"].median())
print("success va collided cung luc (phai = 0):", int(((d["success"] == 1) & (d["collided"] == 1)).sum()))
print(d.groupby("noise")[["success", "collided", "collisions"]].mean().round(3))