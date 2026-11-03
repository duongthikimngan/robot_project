import json

path = r"D:\robot_project\external\RVC3-python\notebooks\chap6.ipynb"
nb = json.load(open(path, encoding="utf-8"))
keys = ("V =", "V=", "P0", "W =", "W=", "robot =",
        "Bicycle", "LandmarkMap", "RangeBearingSensor", "EKF(")

for i, c in enumerate(nb["cells"]):
    src = "".join(c["source"])
    if c["cell_type"] == "code" and any(k in src for k in keys):
        print("--- cell", i)
        print(src)