\# delivery-nav: bám đường (Pure Pursuit cố định/thích nghi) x định vị (dead reckoning/EKF)



\## Môi trường

\- Python 3.12 (pycache cpython-312), pytest 9.1.0. Chạy mọi lệnh từ THƯ MỤC GỐC của repo.

\- Môi trường chính: `conda create -n pyrobotics python=3.12`, rồi `pip install -r env/pyrobotics-requirements.txt`

\- Môi trường RVC3 (chỉ cần cho baseline/rtb\_export.py và các notebook RVC3): `conda env create -f env/rvc3-environment.yml`

\- Kiểm tra: `python -m pytest -q`  (kỳ vọng: 21 passed)



\## Cấu trúc

src/ (thư viện), scripts/ (chạy thí nghiệm, vẽ), tests/, baseline/ (tái hiện \[1]-\[3], đối chiếu RTB),

results/ (kết quả; xem results/INDEX.md), report/, REFERENCES.md.



\## Lệnh một dòng (xem thêm results/INDEX.md)

| Việc | Lệnh |

|---|---|

| Thí nghiệm 2x2 chính (hành lang, lm8, 30 seed) | `python scripts\\run\_experiment.py --tau\_up 8` |

| Quét d cố định (vd d = 2.0) | `python scripts\\run\_experiment.py --d 2.0` |

| Hình Pareto | `python scripts\\plot\_pareto.py` |

| Tầng 2 (landmark) | `python scripts\\run\_experiment.py --lm lm3 --loc ekf --tau\_up 8` (lm3, lm3\_r4, lm8, lm8\_r4) |

| DR riêng | `python scripts\\run\_experiment.py --lm lm8 --loc dr --tau\_up 8` |

| Hình tầng 2 | `python scripts\\plot\_tier2.py` |

| McNemar EKF vs DR | `python -X utf8 scripts\\mcnemar\_tier2.py` |

| Thí nghiệm cổng | `python scripts\\diag\_lm3r4.py --map corridor --lm lm3\_r4 --noise high --seeds 30 --part gate` |

| Phân loại thất bại / tóm tắt cổng | `python scripts\\classify\_failures.py` ; `python scripts\\gate\_summary.py` |

| Chẩn đoán cte theo quãng đường | `python scripts\\diag\_cte\_vs\_s.py` |

| Đối chiếu RTB | `conda activate RVC3` rồi `python baseline\\rtb\_export.py 30`; sau đó `conda activate pyrobotics` rồi `python -X utf8 scripts\\compare\_rtb.py` |



Windows: thêm `-X utf8` khi chuyển hướng đầu ra (`>`) để tránh lỗi mã hóa tiếng Việt.

Đóng ảnh đang mở trước khi chạy lại script (Windows khóa file ảnh).



\## Tái lập

Mọi thí nghiệm dùng seed 0-29; cùng seed dùng chung chuỗi nhiễu cho các tổ hợp (so sánh ghép cặp).

Kết quả đóng băng tại tag `v0.7-results`.

