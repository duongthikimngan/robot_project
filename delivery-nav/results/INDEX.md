\# INDEX kết quả (thư mục results/)

Tất cả thí nghiệm: hành lang, 30 seed, ba mức nhiễu (low/mid/high) trừ khi ghi khác.



| File | Lệnh | Mô tả |

|---|---|---|

| giaidoan5\_experiment/exp\_corridor\_lm8\_d{1,1.5,2,2.65,3.5}.csv | `run\_experiment.py --d X` | 2x2 với d cố định X; nhánh thích nghi gốc (tau\_up = 1) |

| .../exp\_corridor\_lm8\_d1.5\_tu{4,8}.csv | `run\_experiment.py --tau\_up N` | Thích nghi với tau\_up = N (v2 = 8) |

| .../exp\_corridor\_lm8\_d1.5\_tu8\_dronly.csv | `run\_experiment.py --lm lm8 --loc dr --tau\_up 8` | DR riêng, dùng cho McNemar |

| .../exp\_corridor\_lm{3,3\_r4,8\_r4}\_d1.5\_tu8.csv | `run\_experiment.py --lm <cfg> --loc ekf --tau\_up 8` | Tầng 2, EKF |

| .../exp\_{mixed,open}\_lm8\_d1.5\[\_tu8].csv | `run\_experiment.py --map <map> --loc dr --noise low \[--tau\_up 8]` | Kiểm tra tổng quát hóa; CHỈ DR, mức low, 30 seed. Tên file không có `\_dronly` (chạy trước khi thêm hậu tố) |

| .../box\_cte\_\*.png | cùng lệnh với CSV tương ứng | Boxplot cte\_rms |

| .../pareto\_{miss,rms}\_ekf\_v2.png | `plot\_pareto.py` | Pareto steer\_rate với miss\_max / cte\_rms; bản không `\_v2` là bản cũ |

| .../cte\_vs\_s\_mid\_seed0.png | `diag\_cte\_vs\_s.py` | cte và d theo quãng đường, nhiễu mid, seed 0 (1 seed) |

| .../mcnemar\_tier2.txt | `python -X utf8 scripts\\mcnemar\_tier2.py > ...` <kiểm tra lệnh đã dùng> | McNemar và Wilcoxon EKF vs DR |

| fig\_tier2\_success.png, fig\_tier2\_locerr.png | `plot\_tier2.py` | Thành công (Wilson 95%) và sai số định vị |

| fig\_success\_vs\_rreach.png | `plot\_rreach.py` <kiểm tra tham số> | Thành công theo dung sai R\_REACH |

| giaidoan4\_localization/diag\_cover\_\*.csv | `diag\_lm3r4.py --lm <cfg> --noise <n> --part cover` | Phủ sóng landmark dọc tuyến |

| .../diag\_fail\_\*.csv | `diag\_lm3r4.py ... --part fail` | Nguyên nhân thất bại, 30 seed |

| .../diag\_gate\_\*.csv, diag\_gate\_seeds\_\*.csv | `diag\_lm3r4.py ... --part gate` | Thí nghiệm cổng 9.21 / 13.82 / tắt |

| .../ekf\_check\_\*.csv, ekf\_consistency\_\*.png, phaseB\_\*.png | `check\_ekf.py`, `phase\_b.py` <kiểm tra tham số> | Kiểm tra nhất quán EKF (NEES, ±2σ) |

| giaidoan3\_tracking/\*.csv, \*.png | `run\_tracking.py`, `diag\_adaptive.py` <kiểm tra> | Bám đường (pose thật). Các file có `\_hold\_` do bản `pursuit.py` cũ có cơ chế hold; bản hiện tại không có, KHÔNG dùng để tái tạo |

| ../baseline/rtb\_compare\_result.txt | xem README, mục Đối chiếu RTB | EKF nhóm so với RTB, 30 seed |

