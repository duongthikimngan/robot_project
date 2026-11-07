import sys
import pandas as pd

d = pd.read_csv(sys.argv[1])
n = int(sys.argv[2]) if len(sys.argv) > 2 else 10
print("hình dạng:", d.shape)
print("cột:", list(d.columns))
print(d.head(n).to_string())