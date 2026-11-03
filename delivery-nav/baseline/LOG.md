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
