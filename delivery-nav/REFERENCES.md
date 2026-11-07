# REFERENCES

## Nguồn tham khảo
[1] A. Sakai và cộng sự, "PythonRobotics". https://github.com/AtsushiSakai/PythonRobotics
    Commit 8c392897c7f41a6e4d9aab9cb2e9737a7a6fa031 (24/10/2026). Truy cập: 02/11/2026.
    Giấy phép MIT. Copyright (c) 2016 - now Atsushi Sakai and other contributors.
[2] P. Corke, J. Haviland, "Robotics Toolbox for Python". https://github.com/petercorke/robotics-toolbox-python
    Commit 879fb363580840e19ee70b6b175e8cd3543e2b38 (02/11/2026). Truy cập: 02/11/2026.
    Giấy phép MIT. Copyright (c) 2020 Peter Corke and Jesse Haviland.
[3] P. Corke, "RVC3-python". https://github.com/petercorke/RVC3-python
    Commit d97929c33e0ab8f85a2ccc93479edef094df3f40 (13/10/2026). Truy cập: 02/11/2026.
    Giấy phép MIT. Copyright (c) 2021 Peter Corke.
[4] P. Corke, Robotics, Vision and Control, 3rd ed. <điền nếu dùng>.
[5] Slide môn Kỹ thuật Robot, Chương 1-4. <giảng viên, năm>.

## Giấy phép
Ba repo [1]-[3] dùng MIT. Mã trong `src/` và `scripts/` không sao chép nguyên văn từ các repo này.
Nếu có đoạn nào sao chép, giữ nguyên thông báo bản quyền MIT ngay trong file đó và ghi vào bảng bên dưới.
Các file `baseline/rvc3_*_copy.*` là bản sao/chỉnh nhẹ từ [3] để tái hiện hình trong slide
(giữ nguyên thông báo bản quyền của [3]).

## Công cụ AI (khai theo quy định môn học)
Công cụ duy nhất: Claude (Anthropic).
Dùng cho: gợi ý thứ tự và thiết kế thí nghiệm, viết hoặc gợi ý mã script phân tích/vẽ hình,
đọc và diễn giải kết quả, đối chiếu mã RTB, soạn tài liệu.
Nhóm thực hiện: chạy toàn bộ mô phỏng, kiểm tra số liệu, chọn tham số, quyết định kết luận.
Chi tiết từng file xem cột "AI" của bảng dưới.

## Bảng "tham khảo vs tự làm"
| File | Nguồn | Phần sửa đổi / viết mới | AI |
|---|---|---|---|
| src/astar.py | Tự viết (A* 8 hướng, octile) | Chặn cắt góc chéo; đối chiếu hình với baseline [1] | <xác nhận> |
| src/gridmap.py, maps.py | Tự viết | Lưới chiếm dụng, inflate bằng scipy `distance_transform_edt`; 3 bản đồ | <xác nhận> |
| src/smooth.py | Tự viết | Rút gọn line-of-sight, lấy mẫu lại, `line_free` | <xác nhận> |
| src/planner.py | Tự viết | Phồng bản đồ, A* từng cặp điểm giao, `reached` theo pose thật | <xác nhận> |
| src/vehicle.py | Mô hình theo slide §3.1.1 | Cập nhật trung điểm, giới hạn góc lái | <xác nhận> |
| src/pursuit.py | Khung theo slide §3.1.5 [5] | Tự làm: densify, độ cong Menger, look-ahead thích nghi theo độ cong (giảm ngay, tăng theo tau_up). Kết luận về look-ahead của [1]: <ghi từ pp_lookahead_compare.py> | <xác nhận> |
| src/pid.py | Dạng ISA và S-IMC theo slide Ch.3 p2 [5] | Anti-windup, lọc đạo hàm | <xác nhận> |
| src/noise.py | Tự thiết kế | Mô hình nhiễu 4 mức; hiệu chỉnh bằng scripts/calibrate_noise.py | <xác nhận> |
| src/ekf.py | Công thức theo slide Ch.4 [5] | Jacobian, cổng χ², min_range, Joseph, NEES; đối chiếu [2] (xem baseline/rtb_compare_result.txt) | <xác nhận> |
| src/sensor.py, landmarks.py | Tự viết | Range-bearing 360°, che khuất bằng bản đồ thật, RNG riêng | <xác nhận> |
| scripts/run_experiment.py | Tự viết | `--tau_up`, `--loc`, hậu tố tên file, 2x2 ghép cặp | Claude gợi ý các chỗ sửa trong cuộc trò chuyện này |
| scripts/plot_pareto.py, diag_cte_vs_s.py | | | Claude viết (cuộc trò chuyện này) |
| scripts/plot_tier2.py | | Bản hiện tại (Wilson, `--only-success`) đã chỉnh nhiều so với bản đầu | Claude viết bản đầu, nhóm sửa |
| scripts/diag_lm3r4.py | | Docstring ghi "bản tôi đưa" | Do một trợ lý AI cung cấp (<ghi rõ nếu là Claude>) |
| scripts/classify_failures.py, gate_summary.py, mcnemar_tier2.py | | | <xác nhận> |
| baseline/rtb_export.py, scripts/compare_rtb.py | | Phát lại dữ liệu RTB, 30 seed | Claude viết, nhóm chạy và kiểm tra |
| scripts/run_tracking.py | | `<kiểm tra>` | <xác nhận> |
| tests/ (21 test) | | | <xác nhận> |