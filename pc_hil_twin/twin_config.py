"""
twin_config.py - Tệp cấu hình trung tâm cho Giao diện Digital Twin & Mô phỏng HIL 7 tầng.

Chứa toàn bộ hằng số vật lý/thời gian (khớp 1-1 với include/elevator_types.h của STM32),
bảng màu giao diện tối hiện đại (Industrial Slate Theme) và bảng tra trạng thái HSM.
Khi cần đổi số tầng, màu sắc hay ngưỡng an toàn, chỉ cần sửa duy nhất tại file này.
"""

from typing import Any, Dict, Tuple

# ==============================================================================
# 1. HẰNG SỐ KỸ THUẬT ĐỒNG BỘ VỚI FIRMWARE STM32 (elevator_types.h)
# ==============================================================================
MIN_FLOOR: int = 1
MAX_FLOOR: int = 7
NUM_FLOORS: int = 7

# Chu kỳ định thời (mili-giây)
SIM_TICK_MS: int = 50            # Nhịp cập nhật mô phỏng & hoạt ảnh canvas (20 FPS)
UI_REFRESH_MS: int = 50          # Chu kỳ làm mới giao diện Tkinter
TRAVEL_FLOOR_MS: int = 1500      # Thời gian cabin di chuyển hết 1 tầng (1.5 giây)
DOOR_MOVE_MS: int = 3500         # Thời gian đóng hoặc mở hết cánh cửa (3.5 giây)
DOOR_HOLD_MS: int = 3500         # Thời gian giữ cửa mở để khách ra vào (3.5 giây)
CAN_TIMEOUT_MS: int = 2000       # Ngưỡng đứt dây CAN kích hoạt FAIL_SAFE (2 giây)

# Ngưỡng cảm biến an toàn
TEMP_FIRE_THRESH_C: float = 55.0   # >= 55.0 độ C -> kích hoạt báo cháy FIRE_EVAC
TEMP_CLEAR_THRESH_C: float = 48.0  # <= 48.0 độ C -> cho phép hồi phục (Hysteresis)
LOAD_MAX_KG: int = 1000
LOAD_OVERLOAD_KG: int = 800        # >= 800 kg -> giữ cửa mở & báo còi quá tải
LOAD_CLEAR_KG: int = 750           # <= 750 kg -> giải trừ quá tải (Hysteresis 50 kg)

# Danh sách bước chạy kịch bản kiểm thử HIL tự động (Smoke Suite 7 tầng)
HIL_SMOKE_STEPS = [
    ("elev twin on", 0.5, "Bật luồng telemetry $TWIN"),
    ("elev call 6 cabin", 1.2, "Gọi trong cabin lên Tầng 6"),
    ("elev call 3 up", 1.0, "Khách hành lang Tầng 3 gọi LÊN (gom cuốc SCAN)"),
    ("elev call 7 cabin", 1.0, "Bấm thêm Tầng 7 trong cabin"),
    ("elev door obstruct", 2.2, "Thử cảm biến kẹt cửa X0 đảo chiều mở"),
    ("elev fault fire", 3.5, "Tiêm lỗi Hỏa hoạn -> sơ tán thẳng về Tầng 1"),
    ("elev fault clear", 1.2, "Xóa chốt lỗi báo cháy"),
    ("elev fault can_drop", 2.6, "Cắt kết nối CAN -> vào FAIL_SAFE sau 2000 ms"),
    ("elev fault clear", 0.6, "Khôi phục kết nối & đóng cửa an toàn"),
]


# ==============================================================================
# 2. BẢNG MÀU & PHÔNG CHỮ GIAO DIỆN (INDUSTRIAL SLATE DARK THEME)
# ==============================================================================
COLORS: Dict[str, str] = {
    "bg_root":       "#090E1A",  # Nền ngoài cùng tối sâu
    "bg_card":       "#111827",  # Nền các thẻ chức năng
    "bg_card_alt":   "#1A2234",  # Nền phụ cho metric card / thanh tiêu đề
    "bg_shaft":      "#0B1220",  # Nền giếng thang
    "border":        "#263248",  # Viền thẻ tinh tế
    "text_primary":  "#F1F5F9",  # Chữ chính trắng sáng
    "text_muted":    "#94A3B8",  # Chữ phụ xám xanh
    "accent_blue":   "#38BDF8",  # Xanh dương công nghiệp (trạng thái bình thường)
    "accent_indigo": "#6366F1",  # Tím xanh (nút điều khiển chính)
    "accent_green":  "#10B981",  # Xanh ngọc (an toàn / cửa mở / kết nối OK)
    "accent_amber":  "#F59E0B",  # Hổ phách (cảnh báo / đang chờ gọi tầng)
    "accent_orange": "#EA580C",  # Cam đậm (quá tải / đứt CAN)
    "accent_red":    "#EF4444",  # Đỏ cảnh báo (hỏa hoạn / E-Stop)
    "btn_idle":      "#1E293B",  # Nền nút chưa kích hoạt
    "btn_cabin_on":  "#D97706",  # Đèn nút Cabin đã bấm
    "btn_up_on":     "#059669",  # Đèn nút Hành lang LÊN đã bấm
    "btn_dn_on":     "#0284C7",  # Đèn nút Hành lang XUỐNG đã bấm
    "console_bg":    "#060911",  # Nền cửa sổ Console Log
}

FONTS = {
    "title":      ("Segoe UI", 13, "bold"),
    "section":    ("Segoe UI", 10, "bold"),
    "body":       ("Segoe UI", 9),
    "body_bold":  ("Segoe UI", 9, "bold"),
    "mono":       ("Consolas", 9),
    "mono_bold":  ("Consolas", 10, "bold"),
    "metric":     ("Consolas", 13, "bold"),
    "seg7":       ("Consolas", 18, "bold"),
}


# ==============================================================================
# 3. BẢNG TRA TRẠNG THÁI HSM -> NHÃN TIẾNG VIỆT, MÀU SẮC & KÝ TỰ LED 7 ĐOẠN
# ==============================================================================
# Dùng dictionary ánh xạ trực tiếp (table-driven) thay vì if/elif dài dòng
STATE_META: Dict[str, Tuple[str, str, str]] = {
    "INIT":         ("KHỞI TẠO",       COLORS["text_muted"],   "--"),
    "IDLE_CLOSED":  ("ĐỖ CHỜ LỆNH",    COLORS["accent_green"], "- "),
    "DOOR_OPENING": ("ĐANG MỞ CỬA",    COLORS["accent_blue"],  "OP"),
    "DOOR_OPEN":    ("CỬA ĐANG MỞ",    COLORS["accent_green"], "[]"),
    "DOOR_CLOSING": ("ĐANG ĐÓNG CỬA",  COLORS["accent_blue"],  "CL"),
    "MOVING_UP":    ("ĐANG CHẠY LÊN",  COLORS["accent_blue"],  "^▲"),
    "MOVING_DOWN":  ("ĐANG CHẠY XUỐNG", COLORS["accent_amber"], "v▼"),
    "FIRE_EVAC":    ("SƠ TÁN CHÁY",    COLORS["accent_red"],   "FE"),
    "FAIL_SAFE":    ("AN TOÀN SỰ CỐ",  COLORS["accent_orange"], "E-"),
}


def default_telemetry() -> Dict[str, Any]:
    """Tạo bản sao gói trạng thái telemetry mặc định (dùng chung cho cả Live và Simulator)."""
    return {
        "ms": 0,
        "st": "IDLE_CLOSED",
        "fl": 1,
        "pos_f": 1.0,       # Vị trí nội suy (1.0..7.0) để vẽ cabin trượt mượt trong Offline Sim
        "tgt": 0,
        "dir": 0,
        "door": 0,
        "y0": 0,
        "y1": 0,
        "fire": 0,
        "estop": 0,
        "ovl": 0,
        "canf": 0,
        "temp": 270,        # 27.0 độ C (đơn vị 0.1 độ C giống STM32)
        "load": 150,        # 150 kg
        "tec": 0,
        "rec": 0,
        "lat_us": 0,
        "cm": 0,            # cabin_mask: bit 0..6 ứng với Tầng 1..7
        "um": 0,            # hall_up_mask: bit 0..5 ứng với Tầng 1..6
        "dm": 0,            # hall_down_mask: bit 1..6 ứng với Tầng 2..7
        "trips": 0,         # Tổng số tầng đã đi qua
        "obstruct_flash": 0 # Cờ nháy thanh IR chống kẹt trên canvas
    }
