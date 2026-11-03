import sys

path, a, b = sys.argv[1], int(sys.argv[2]), int(sys.argv[3])
with open(path, encoding="utf-8") as f:
    for i, line in enumerate(f, 1):
        if a <= i <= b:
            print(i, line.rstrip())