# Báo cáo Bài tập lớn -- Hệ thống Nhúng (CO3053)

Thư mục mã nguồn LaTeX cho báo cáo đề tài **"Hệ thống Điều khiển Thang máy 7 tầng An toàn Thời gian thực trên Zephyr RTOS, CAN Bus và Kiểm thử HIL"** (Trường ĐH Bách Khoa -- ĐHQG-HCM, Lớp `L01`, HK261).

## Cấu trúc thư mục
- `main.tex`: File gốc cấu hình font tiếng Việt T5, style bảng biểu và style sơ đồ TikZ IEEE.
- `cover.tex`: Trang bìa chuẩn HCMUT.
- `sections/`: Nội dung 5 chương (`01_tong_quan.tex` .. `05_ket_luan.tex`) và `assignment.tex`.
- `figures/`: 8 sơ đồ vector TikZ (`fig_vmodel.tex`, `fig_sys_arch.tex`, `fig_shared_buses.tex`, `fig_dt_conflict.tex`, `fig_rtos_arch.tex`, `fig_hsm_states.tex`, `fig_event_convergence.tex`, `fig_can_eeprom.tex`).
- `image/`: Logo trường, bản đồ kit BKIT ARM4, ảnh GUI Digital Twin và đồ thị thực nghiệm.
- `main.pdf`: Bản PDF đã biên dịch hoàn chỉnh.

## Biên dịch
```bash
cd report
pdflatex -interaction=nonstopmode main.tex
pdflatex -interaction=nonstopmode main.tex
```
