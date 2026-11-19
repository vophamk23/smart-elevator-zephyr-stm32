"""
elevator_twin.py - Điểm khởi chạy (Entrypoint) của PC Digital Twin & HIL Simulator 7 Tầng.

Toàn bộ logic được phân tách sạch sẽ sang các module chuyên biệt để dễ bảo trì và debug:
  - `twin_config.py`    : Hằng số hệ thống 7 tầng, bảng màu Industrial Theme, bảng tra trạng thái.
  - `twin_scheduler.py` : Bộ lập lịch hai chiều SCAN/LOOK dùng bitmask 7 bit.
  - `twin_simulator.py` : Máy trạng thái HSM 7 tầng chạy Offline trên PC khi chưa cắm STM32.
  - `twin_serial.py`    : Luồng đọc RS232 UART không chặn, tách `$TWIN` JSON & xuất file CSV.
  - `twin_canvas.py`    : Mô hình vẽ giếng thang 7 tầng, cáp treo, đối trọng & LED 7 đoạn ảo.
  - `twin_ui.py`        : Giao diện chính 3 cột, nút bấm 7 tầng, thanh kéo cảm biến & Console CLI.

Cách chạy:
  python elevator_twin.py
"""

import os
import sys
import tkinter as tk

# Bảo đảm luôn import được các module anh em dù chạy từ thư mục gốc hay thư mục pc_hil_twin
CURRENT_DIR = os.path.dirname(os.path.abspath(__file__))
if CURRENT_DIR not in sys.path:
    sys.path.insert(0, CURRENT_DIR)

from twin_ui import ElevatorTwinWindow


def main() -> None:
    """Khởi tạo cửa sổ gốc Tkinter và chạy vòng lặp sự kiện giao diện."""
    root = tk.Tk()
    ElevatorTwinWindow(root)
    root.mainloop()


if __name__ == "__main__":
    main()
