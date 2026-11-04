\# Pipeline delivery-nav (chot ngay 2026-11-03)



\## Giai doan 2 (ke hoach, chay mot lan truoc khi robot chay)

ban do luoi -> phong vat can (inflate) -> A\* giua tung cap diem giao

\-> noi cac doan -> rut gon diem thang hang / line-of-sight -> noi suy min

\-> danh sach waypoint cuoi



\## Vong lap mo phong (Giai doan 5), moi buoc dt

1\. Robot that di chuyen theo (v, psi) + nhieu

2\. Sinh odometry nhieu + quan sat landmark (gioi han tam nhin, nhieu range-bearing)

3\. Bo dinh vi (dead reckoning hoac EKF) -> pose uoc luong

4\. Pure Pursuit tinh lenh moi tu pose uoc luong

5\. Ghi log: pose that, pose uoc luong, P, lenh, d, khoang cach toi vat can



\## Quy uoc

\- Cross-track error tinh tu pose THAT toi doan duong gan nhat.

\- Den diem giao khi pose THAT cach < 0.5 m.

\- Cung seed nhieu cho moi to hop A1, A2, B1, B2 (so sanh ghep cap).

\- Moi mo hinh xe: bicycle dong hoc voi gioi han goc lai.

