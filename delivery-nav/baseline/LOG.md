\# LOG chay baseline



\## Phien ban goi da cai (moi truong RVC3)

\- rvc3python: 1.0.0 (conda-forge)

\- roboticstoolbox-python: 1.4.2

\- Python 3.10.22, numpy 2.2.6, matplotlib 3.10.9



\## Ten lop da kiem tra import OK

Bicycle, LandmarkMap, RangeBearingSensor, EKF (from roboticstoolbox)

\## 2026-11-03 | <Ngan Duong> | Notebook RVC3 chap4

\- Nguon: RVC3-python/notebooks/chap4.ipynb (lanechange, drivepoint, drivepursuit), commit d97929c, goi rvc3python 1.0.0

\- Ket qua: chay het cac o, khong loi

\- Anh: (chua luu)  | Nhan xet: (so voi slide 3.1.2, 3.1.3, 3.1.5)



\## 2026-11-03 | <Ngan Duong> | drivepoint tu viet (mine\_drivepoint\_Kv-Kh-compare\_2026-11-03.png)

\- Nguon: baseline/drivepoint\_check.py (tu viet, numpy)

\- Ket qua: v dau = 1.5 / 1.5 / 12.0; lenh lai dau = 3.14 / 6.28 / 3.14 -> KHOP slide 3.1.3

\- Khac slide: quy dao xy gan thang roi quay gap o cuoi, do gia dinh L=1, psi\_max=0.45\*pi (chua kiem chung). Can doi theo RVC3/models/drivepoint.py



\## 2026-11-03 | <Ngan Duong> | Pure Pursuit PythonRobotics

\- Nguon: PathTracking/pure\_pursuit/pure\_pursuit.py, commit 8c39289

\- DOC CODE: dong 118 Lf = k\*state.v + Lfc, k=0.1, Lfc=2.0 -> look-ahead DA phu thuoc van toc

&#x20; => "thich nghi theo van toc" khong con la cai tien moi; chon huong 1 (theo do cong)

\- Khac slide: luat lai delta = atan2(2\*WB\*sin(alpha)/Lf,1) (dong 146); toc do dieu khien P (Kp=1), WB=2.9, MAX\_STEER=pi/4

\- Chay: baseline/pp\_lookahead\_compare.py (import file goc, dat k=0, Lfc=d)

\- Ket qua (sai so TB / max / thoi gian):

&#x20; mac dinh 0.70/3.00/40.4s | d=1 2.16/4.80/64s (mat on dinh, loi chi so) | d=5 0.89/3.01/38.4s | d=10 2.32/3.97/30.6s

&#x20; (max \~3 m la do lech xuat phat y=-3)

\- Nhan xet: d=10 muot nhung cat goc (khop slide). d=1 KHONG khop slide (slide: bam sat nhung giat); nguyen nhan co the: d < WB=2.9 nen do loi lai lon.

\- Canh bao: EKF PythonRobotics co ComplexWarning o utils/plot.py dong 38-40 (numpy 2.5.3), khong sua ma goc.
## 2026-11-03 | <ten ban> | Kiem chung tham so xe
- Nguon: help(bd.BICYCLE) trong bdsim (RVC3), mac dinh L=1, steer_max=0.45*pi, speed_max/accel_max=inf
- drivepoint_check.py dung dung L=1, psi_max=0.45*pi -> tham so da kiem chung
- drivepoint.py cua tac gia: pgoal=[5,5], qs=[5,2,0], Kv=0.5, Kh=1.5 (KHAC slide: qs=[8,5,pi/2], Kh=2)
- EKF PythonRobotics: hoat hinh hien, co ComplexWarning vo hai o utils/plot.py
- Chua giai thich: quy dao xy cua script (y max ~5.3) khong cong nhu hinh slide (~5.6)
## 2026-11-03 | <ten ban> | move-to-pose (driveconfig, 3.1.6)
- Nguon: baseline/rvc3_driveconfig_copy.py (tu RVC3/models/driveconfig.py, commit d97929c)
- Tham so: xg=[5,5,pi/2], Krho=1, Kalpha=5, Kbeta=-2, speed_max=2, steer_max=1.3
- Chay 1: x0=[5,9,0] (mac dinh tac gia). Dung do rho<0.01 luc t~7.9 s; beta ve ~0; alpha ~0.17 luc dung -> DEN DUNG TU THE DICH
- Chay 2: x0=[9,5,0] (theo slide): ket qua <dien sau khi chay>
- Nhiễu odometry (ĐÃ XÁC NHẬN, Vehicle.py dòng 773-785): trạng thái thật cập nhật
  chính xác theo lệnh; odo = (‖Δxy‖, Δθ) từ trạng thái thật, rồi cộng N(0, V) mỗi bước
  (dt = 0,1 s). Nhiễu chỉ ở số đo odometry, không ở chuyển động thật.
- Hạn chế: nhiễu cộng tuyệt đối nên pose ước lượng trôi cả khi xe đứng yên;
  odo[0] là chuẩn (không biểu diễn chạy lùi).
- Hệ quả cho đề tài: mô hình nhiễu của nhóm dùng nhiễu tỉ lệ quãng đường
  (xem Giai đoạn 4), hiệu chỉnh theo sai số dead reckoning cuối tuyến.
## EKF RVC3: dead reckoning (nhóm A) và định vị theo landmark (nhóm B)
- Ngày / người chạy: 2026-11-03, <tên>
- Nguồn: RVC3-python, notebooks\chap6.ipynb, ô 6, 7, 15, 16 (A) và ô 30 (B), commit <hash>
- Môi trường: RVC3 (rvc3python 1.0.0, roboticstoolbox-python 1.4.2, Python 3.10.22)
- Lệnh chạy: notebook chap6 (bản chép) hoặc python rvc3_ekf_demo.py A | B
- Tham số: V = diag(0,02 m ; 0,5°)², P0 = diag(0,05 ; 0,05 ; 0,5°)², W = diag(0,1 m ; 1°)²
  cảm biến: góc [-90°, 90°], tầm 4 m, đọc mỗi bước (0,1 s), mỗi lần một landmark
  bản đồ 20 landmark, workspace ±10 m (20 x 20 m), T = 20 s
- Bicycle mặc định: L = 1 m, steer_max = 1,41 rad, không giới hạn tốc độ và gia tốc
- Đã chỉnh so với notebook gốc: thêm ekf.run(T=20) và savefig; trong script thêm seed=0 cho
  nhóm A, plot=False cho cảm biến; không sửa mã gốc trong external\
- Ảnh: rvc3_ekf_deadreckoning_xy / _pnorm; rvc3_ekf_landmark_xy / _pnorm (2026-11-03)
- Đã xác nhận (Vehicle.py dòng 773-785): xe thật đi đúng, nhiễu chỉ ở số đo odometry,
  mẫu mới mỗi bước, dt = 0,1 s
- Số liệu nhóm A (một lần chạy, chưa cố định seed trong notebook): sau 150 bước,
  σ_x = 0,41 m, σ_y = 0,32 m, σ_θ ≈ 6,2° (khớp 0,5°·√150 ≈ 6,1°)
- Kiểm thử đối chứng cho mã tự viết ở Giai đoạn 3-4:
  step((1, 0.3)) từ [0,0,0] → q = [0.1, 0, 0.03093];
  f([0,0,0], [0.1025, 0.02978]) → [0.1025, 0, 0.02978];
  Fx([0,0,0], [0.5, 0.1]) → ma trận đơn vị, phần tử (2,3) = 0,5
- Hạn chế: nhóm A và B là hai robot khác nhau nên chỉ minh họa hiện tượng, không so định lượng;
  cảm biến chỉ thấy ~1 landmark mỗi lúc (mật độ 0,05 landmark/m², tầm 4 m, nửa vòng)
- Đối chiếu slide Ch.4 §4.5: <khớp / không khớp. Ghi dạng Pnorm quan sát được:
  nhóm A tăng dần?, nhóm B răng cưa?>

## A* PythonRobotics
- Nguon: PathPlanning/AStar/a_star.py, commit 8c39289
- Chay: python a_star.py (moi truong pyrobotics), anh pyrobotics_astar_default_2026-11-03.png
- Tham so doc tu code: grid_size=<..>, robot_radius=<..>, so huong di chuyen=<..>
- Nhan xet: duong di <co/khong> vong quanh vat can; <co bac thang/goc vuong khong>
- Ghi chu cho Giai doan 2: duong A* luoi can lam muot truoc khi dua vao Pure Pursuit

## Giai doan 2 - A* tu viet (2026-11-04)
- Nguon: viet moi, src/astar.py (tham khao y tuong A* chung, khong sao chep ma)
- Thiet ke: 8 huong, chi phi cheo sqrt(2), khong cat goc, heuristic octile, chay tren ban do da phong 1.0 m
- Kiem thu: experiments/test_astar.py -> <ket qua that>
- Tuyen 6 diem giao: <so diem>, <tong do dai> m; anh results/astar_raw_corridor_2026-11-04.png
- Nhan xet: <co bac thang/goc vuong o dau>

## Giai doan 2 - lam muot (2026-11-04)
- src/smooth.py: line-of-sight tham lam + noi suy deu 0.5 m; ghim 2 dau ve diem giao chinh xac
- Ket qua: 93.36 m -> 92.30 m; 188 -> 190 diem; test_smooth.py PASS
- Anh: results/astar_raw_corridor_2026-11-04.png, results/astar_smooth_corridor_2026-11-04.png
- Nhan xet: <mo ta hinh sau lam muot, doan 1->2 con gap khuc may cho>
- Han che: goc vuong o diem giao 3 va 4 con nguyen, chua bo tron
