\# Tai lieu tham khao



\[1] PythonRobotics

&#x20;   Tac gia: Atsushi Sakai va cong dong

&#x20;   URL: https://github.com/AtsushiSakai/PythonRobotics

&#x20;   Commit: 8c392897c7f41a6e4d9aab9cb2e9737a7a6fa031 | Truy cap: 2026-11-02

&#x20;   Giay phep: MIT

&#x20;   Dung cho: tham khao Pure Pursuit, EKF, A\*; doi chieu ket qua



\[2] Robotics Toolbox for Python (RTB)

&#x20;   Tac gia: Peter Corke, Jesse Haviland

&#x20;   URL: https://github.com/petercorke/robotics-toolbox-python

&#x20;   Commit: 879fb363580840e19ee70b6b175e8cd3543e2b38 | Goi: roboticstoolbox-python 1.4.2

&#x20;   Giay phep: MIT

&#x20;   Dung cho: Bicycle, LandmarkMap, RangeBearingSensor, EKF (doi chieu)



\[3] RVC3-python

&#x20;   Tac gia: Peter Corke

&#x20;   URL: https://github.com/petercorke/RVC3-python

&#x20;   Commit: d97929c33e0ab8f85a2ccc93479edef094df3f40 | Goi: rvc3python 1.0.0

&#x20;   Giay phep: <kiem tra file LICENSE trong repo>

&#x20;   Dung cho: mo hinh xe (lanechange, drivepoint, driveline, driveconfig, drivepursuit), EKF chap6



\[4] Slide mon hoc Ch.1-4

&#x20;   Dung cho: co so ly thuyet va hinh doi chieu
# REFERENCES

## Nguồn tham khảo (không chép nguyên mã trừ khi ghi rõ ở bảng bên dưới)

[1] A. Sakai et al., PythonRobotics. https://github.com/AtsushiSakai/PythonRobotics
    Commit 8c392897c7f41a6e4d9aab9cb2e9737a7a6fa031 (24/10/2026). Truy cập 03/11/2026. Giấy phép: <MIT?>.

[2] P. Corke, Robotics Toolbox for Python. https://github.com/petercorke/robotics-toolbox-python
    Commit 879fb363580840e19ee70b6b175e8cd3543e2b38 (02/11/2026). Truy cập 03/11/2026. Giấy phép: <MIT?>.

[3] P. Corke, RVC3-python. https://github.com/petercorke/RVC3-python
    Commit d97929c33e0ab8f85a2ccc93479edef094df3f40 (13/10/2026). Truy cập 03/11/2026. Giấy phép: <MIT?>.

[4] P. Corke, Robotics, Vision and Control, 3rd ed. (sách) <điền nếu dùng>.
[5] Slide môn Kỹ thuật Robot, Ch.1 đến Ch.4 <điền tên giảng viên, năm>.

## Công cụ AI
<Điền đúng sự thật: tên công cụ, dùng cho việc gì (ví dụ: gợi ý cấu trúc thí nghiệm,
viết script phân tích/vẽ hình, đọc kết quả), và phần nào nhóm tự viết/kiểm chứng.>

## Bảng "tham khảo vs tự làm"

| File | Nguồn | Phần sửa đổi | Phần viết mới |
|---|---|---|---|
| src/astar.py | <tự viết / tham khảo [1] astar.py> | <...> | <...> |
| src/gridmap.py, maps.py | <...> | | |
| src/smooth.py | <...> | | |
| src/vehicle.py | <tham khảo slide §3.1.1> | | |
| src/pursuit.py | <tham khảo [1] pure_pursuit; look-ahead thích nghi theo độ cong: tự làm> | | |
| src/pid.py | <...> | | |
| src/noise.py | <...> | | |
| src/ekf.py | <tự cài; đối chiếu [2]> | | |
| src/sensor.py, landmarks.py | <...> | | |
| src/planner.py | <...> | | |
| scripts/, tests/ | <...> | | |
# REFERENCES

## Nguồn tham khảo

[1] A. Sakai và cộng sự, "PythonRobotics". https://github.com/AtsushiSakai/PythonRobotics
    Commit 8c392897c7f41a6e4d9aab9cb2e9737a7a6fa031 (24/10/2026). Truy cập: <ngày truy cập>.
    Giấy phép: MIT. Copyright (c) 2016 - now Atsushi Sakai and other contributors.

[2] P. Corke và J. Haviland, "Robotics Toolbox for Python". https://github.com/petercorke/robotics-toolbox-python
    Commit 879fb363580840e19ee70b6b175e8cd3543e2b38 (02/11/2026). Truy cập: <ngày truy cập>.
    Giấy phép: MIT. Copyright (c) 2020 Peter Corke and Jesse Haviland.

[3] P. Corke, "RVC3-python". https://github.com/petercorke/RVC3-python
    Commit d97929c33e0ab8f85a2ccc93479edef094df3f40 (13/10/2026). Truy cập: <ngày truy cập>.
    Giấy phép: MIT. Copyright (c) 2021 Peter Corke.

[4] P. Corke, Robotics, Vision and Control, 3rd ed. <điền nếu dùng>.
[5] Slide môn Kỹ thuật Robot, Chương 1 đến 4. <giảng viên, năm>.

## Giấy phép
Cả ba repo [1]-[3] dùng giấy phép MIT. Nếu sao chép đoạn mã, giữ nguyên thông báo
bản quyền và giấy phép MIT trong file đó. Bảng bên dưới ghi rõ file nào có sao chép/chỉnh sửa.

