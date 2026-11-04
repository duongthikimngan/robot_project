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





\## Vòng lặp một bước thời gian (Giai đoạn 4-5)

1\. Bộ bám đường nhận pose ĐIỀU KHIỂN q\_ctrl và cho lệnh (v, ψ).

2\. Xe THẬT cập nhật chính xác theo lệnh (có giới hạn v, ψ): q\_true.

3\. Odometry thật (d, dθ) lấy từ thay đổi của q\_true; thêm nhiễu → odometry ĐO (d\_m, dθ\_m).

4\. Bộ định vị cập nhật từ odometry đo:

&#x20;  - dead reckoning: q\_est = f(q\_est, d\_m, dθ\_m)

&#x20;  - EKF: dự đoán bằng odometry đo, cập nhật bằng quan sát landmark

5\. q\_ctrl = q\_est (chế độ dr / ekf) hoặc q\_true (chế độ truth, chỉ để hiệu chỉnh).

Quy tắc: cte, va chạm, tỉ lệ giao hàng tính trên q\_true. Logic chuyển điểm giao kế tiếp

của bộ bám dùng q\_ctrl, KHÔNG dùng q\_true.

