import os
import pandas as pd

OUT = r"results\giaidoan4_localization"
rows = []
for lm, noise in (("lm3_r4", "mid"), ("lm3_r4", "high"), ("lm8_r4", "high")):
    for g in ("9.21", "13.82", "off"):
        p = rf"{OUT}\diag_gate_seeds_corridor_{lm}_{noise}_{g}.csv"
        if not os.path.exists(p):
            continue
        d = pd.read_csv(p)
        rows.append(dict(
            cau_hinh=f"{lm} {noise}", cong=g,
            thanh_cong=int(d.success.sum()),
            nees_trung_vi=round(d.nees_mean.median(), 2),
            ti_le_nees_le_4_5=round((d.nees_mean <= 4.5).mean(), 2),
            nees_tb_bo_max=round(d.nees_mean.nsmallest(len(d) - 1).mean(), 2),
            sai_so_cuoi_trung_vi=round(d.loc_err_end.median(), 2) if "loc_err_end" in d.columns else None,
        ))
print(pd.DataFrame(rows).to_string(index=False))