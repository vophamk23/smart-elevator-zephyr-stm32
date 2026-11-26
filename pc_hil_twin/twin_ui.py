"""
twin_ui.py - Cửa sổ Giao diện Chính 3 Cột cho Digital Twin & HIL Thang máy 7 Tầng.

Chức năng tổng quát của file:
  - `twin_canvas.py`   : Cột trái vẽ giếng thang 7 tầng + đèn LED 7 đoạn ảo.
  - Bảng điều khiển giữa: Ma trận nút gọi 7 tầng (Cabin + Hành lang Lên/Xuống), cửa, tiêm lỗi HIL, thanh kéo cảm biến Offline.
  - Dashboard phải     : 8 thẻ thông số thời gian thực + thanh tiến độ mở cửa + Shell Console tô màu theo loại log.

Các phương thức quan trọng nhất trong file:
  - `ElevatorTwinApp.dispatch_cmd()`    : Gửi lệnh điều khiển xuống bo mạch BKIT thật qua UART (hoặc vào bộ mô phỏng Offline).
  - `ElevatorTwinApp._ui_refresh_loop()`: Vòng lặp 20 FPS đọc gói tin `$TWIN` mới nhất và cập nhật toàn bộ đồ họa & thông số.
  - `ElevatorTwinApp._run_smoke_step()` : Chạy tự động chuỗi kịch bản kiểm thử HIL 7 tầng (Smoke Test Suite).
"""

import time
import tkinter as tk
from tkinter import ttk
from typing import Any, Dict, Tuple

from twin_canvas import ElevatorShaftCanvas
from twin_config import (
    COLORS,
    FONTS,
    MAX_FLOOR,
    MIN_FLOOR,
    SIM_TICK_MS,
    STATE_META,
    UI_REFRESH_MS,
    default_telemetry,
)
from twin_serial import SerialBridge
from twin_simulator import OfflineElevatorSimulator


class ElevatorTwinWindow:
    """Bộ điều khiển giao diện chính (Controller + View) cho Digital Twin 7 tầng."""

    def __init__(self, root: tk.Tk) -> None:
        """Khởi tạo cầu nối Serial, bộ mô phỏng Offline và dựng toàn bộ giao diện 3 cột."""
        self.root = root
        self.root.title("BKIT ARM4 • Hệ thống Thang máy 7 Tầng (Digital Twin & HIL)")
        self.root.geometry("1220x740")
        self.root.minsize(1060, 640)
        self.root.configure(bg=COLORS["bg_root"])

        self.serial = SerialBridge()
        self.sim = OfflineElevatorSimulator(log_cb=self.append_log)

        # Mặc định chạy Offline Simulator để người dùng test ngay 7 tầng dù chưa cắm kit
        self.live_mode = False
        self.latest_tel: Dict[str, Any] = default_telemetry()
        self._last_sim_record_ms = 0
        self._temp_tx_after_id = None
        self._load_tx_after_id = None
        self._suppress_slider_cb = False
        self._last_user_slider_ts = 0.0
        self._last_live_rx_ts = 0.0
        self._last_keepalive_tx_ts = 0.0
        self._live_pos_f = 1.0
        self._live_door_pct = 0.0

        # Lưu tham chiếu các nút gọi tầng để đổi màu sáng/tắt theo bitmask hàng đợi
        self.floor_btns: Dict[Tuple[str, int], tk.Button] = {}
        self.metric_labels: Dict[str, tk.Label] = {}

        self._build_header()
        self._build_main_columns()

        self.append_log("OK", "Khởi động giao diện 7 tầng thành công (chế độ OFFLINE SIMULATOR).")
        self.append_log("INFO", "Bạn có thể bấm gọi tầng F1..F7 ngay, hoặc cắm cáp RS232 để chuyển sang LIVE STM32.")

        self._tick_loop()

    # ==========================================================================
    # 1. XÂY DỰNG CÁC KHỐI GIAO DIỆN (UI BUILDERS)
    # ==========================================================================
    def _make_card(self, parent: tk.Misc, title: str, title_color: str = COLORS["accent_blue"]) -> tk.Frame:
        """Hàm tiện ích tạo khung thẻ card đồng nhất (tránh lặp code styling ở nhiều nơi)."""
        outer = tk.Frame(
            parent,
            bg=COLORS["bg_card"],
            highlightthickness=1,
            highlightbackground=COLORS["border"],
            padx=10,
            pady=8,
        )
        outer.pack(fill=tk.X, pady=(0, 8))
        tk.Label(
            outer,
            text=title,
            fg=title_color,
            bg=COLORS["bg_card"],
            font=FONTS["section"],
            anchor="w",
        ).pack(fill=tk.X, pady=(0, 6))
        return outer

    def _build_header(self) -> None:
        """Dựng thanh công cụ phía trên: chọn chế độ Offline/Live, COM Port, Auto Demo & CSV."""
        bar = tk.Frame(
            self.root,
            bg=COLORS["bg_card"],
            highlightthickness=1,
            highlightbackground=COLORS["border"],
            padx=12,
            pady=8,
        )
        bar.pack(fill=tk.X, padx=10, pady=(10, 6))

        # Tên hệ thống bên trái
        tk.Label(
            bar,
            text="BKIT ARM4  |  7-FLOOR SMART ELEVATOR HIL",
            fg=COLORS["text_primary"],
            bg=COLORS["bg_card"],
            font=FONTS["title"],
        ).pack(side=tk.LEFT)

        # Nút gạt chuyển nhanh Offline Sim / Live STM32
        self.mode_badge = tk.Label(
            bar,
            text="● OFFLINE SIMULATOR",
            fg="#052E16",
            bg=COLORS["accent_green"],
            font=FONTS["body_bold"],
            padx=10,
            pady=4,
        )
        self.mode_badge.pack(side=tk.RIGHT, padx=(8, 0))

        # Nhóm nút thao tác & cổng nối tiếp RS232
        controls = tk.Frame(bar, bg=COLORS["bg_card"])
        controls.pack(side=tk.RIGHT)

        tk.Label(controls, text="Cổng COM:", fg=COLORS["text_muted"], bg=COLORS["bg_card"], font=FONTS["body"]).pack(
            side=tk.LEFT, padx=(0, 4)
        )
        ports = self.serial.list_ports()
        self.port_var = tk.StringVar(value=ports[0])
        self.port_combo = ttk.Combobox(controls, textvariable=self.port_var, values=ports, width=9)
        self.port_combo.pack(side=tk.LEFT, padx=(0, 6))

        self.btn_conn = tk.Button(
            controls,
            text="Kết nối STM32",
            command=self._toggle_serial_connection,
            bg=COLORS["accent_indigo"],
            fg="#FFFFFF",
            font=FONTS["body_bold"],
            relief="flat",
            padx=10,
            pady=4,
            cursor="hand2",
        )
        self.btn_conn.pack(side=tk.LEFT, padx=3)

        tk.Button(
            controls,
            text="Auto Demo 7T",
            command=self._start_smoke_suite,
            bg="#2563EB",
            fg="#FFFFFF",
            font=FONTS["body_bold"],
            relief="flat",
            padx=10,
            pady=4,
            cursor="hand2",
        ).pack(side=tk.LEFT, padx=3)

        tk.Button(
            controls,
            text="Xuất CSV",
            command=self._export_csv,
            bg=COLORS["btn_idle"],
            fg=COLORS["text_primary"],
            font=FONTS["body_bold"],
            relief="flat",
            padx=10,
            pady=4,
            cursor="hand2",
        ).pack(side=tk.LEFT, padx=3)

        tk.Button(
            controls,
            text="Làm mới COM",
            command=self._refresh_ports,
            bg=COLORS["btn_idle"],
            fg=COLORS["text_muted"],
            font=FONTS["body"],
            relief="flat",
            padx=8,
            pady=4,
            cursor="hand2",
        ).pack(side=tk.LEFT, padx=3)

    def _build_main_columns(self) -> None:
        """Chia bố cục chính thành 3 cột cân đối (Giếng thang | Điều khiển | Dashboard & Log)."""
        body = tk.Frame(self.root, bg=COLORS["bg_root"])
        body.pack(fill=tk.BOTH, expand=True, padx=10, pady=(0, 10))
        body.grid_columnconfigure(0, weight=0, minsize=320)
        body.grid_columnconfigure(1, weight=1, minsize=390)
        body.grid_columnconfigure(2, weight=1, minsize=430)
        body.grid_rowconfigure(0, weight=1)

        # Cột 0: Giếng thang 7 tầng
        self.shaft = ElevatorShaftCanvas(body)
        self.shaft.grid(row=0, column=0, sticky="nsew", padx=(0, 8))

        # Cột 1: Ma trận nút gọi 7 tầng + Điều khiển cửa + Tiêm lỗi & Cảm biến
        mid_col = tk.Frame(body, bg=COLORS["bg_root"])
        mid_col.grid(row=0, column=1, sticky="nsew", padx=(0, 8))
        self._build_floor_matrix_card(mid_col)
        self._build_door_card(mid_col)
        self._build_hil_and_sensors_card(mid_col)

        # Cột 2: Thẻ thông số Telemetry + Thanh tiến độ mở cửa + Shell Console
        right_col = tk.Frame(body, bg=COLORS["bg_root"])
        right_col.grid(row=0, column=2, sticky="nsew")
        self._build_metrics_card(right_col)
        self._build_console_card(right_col)

    def _build_floor_matrix_card(self, parent: tk.Misc) -> None:
        """Tạo bảng nút gọi 7 tầng từ Tầng 7 xuống Tầng 1 (giống bảng điều khiển thật)."""
        card = self._make_card(parent, "BẢNG GỌI 7 TẦNG (CABIN & HÀNH LANG)", COLORS["accent_blue"])

        for fl in range(MAX_FLOOR, MIN_FLOOR - 1, -1):
            row = tk.Frame(card, bg=COLORS["bg_card"])
            row.pack(fill=tk.X, pady=2)

            tk.Label(
                row,
                text=f"Tầng {fl}",
                fg=COLORS["text_muted"],
                bg=COLORS["bg_card"],
                font=FONTS["mono_bold"],
                width=7,
                anchor="w",
            ).pack(side=tk.LEFT)

            btn_cab = tk.Button(
                row,
                text=f"Cabin F{fl}",
                command=lambda f=fl: self.dispatch_command(f"elev call {f} cabin"),
                bg=COLORS["btn_idle"],
                fg=COLORS["text_primary"],
                font=FONTS["body_bold"],
                relief="flat",
                width=10,
                pady=3,
                cursor="hand2",
            )
            btn_cab.pack(side=tk.LEFT, padx=3)
            self.floor_btns[("cabin", fl)] = btn_cab

            if fl < MAX_FLOOR:
                btn_up = tk.Button(
                    row,
                    text="▲ Lên",
                    command=lambda f=fl: self.dispatch_command(f"elev call {f} up"),
                    bg=COLORS["btn_idle"],
                    fg=COLORS["accent_green"],
                    font=FONTS["body_bold"],
                    relief="flat",
                    width=8,
                    pady=3,
                    cursor="hand2",
                )
                btn_up.pack(side=tk.LEFT, padx=3)
                self.floor_btns[("up", fl)] = btn_up

            if fl > MIN_FLOOR:
                btn_dn = tk.Button(
                    row,
                    text="▼ Xuống",
                    command=lambda f=fl: self.dispatch_command(f"elev call {f} down"),
                    bg=COLORS["btn_idle"],
                    fg=COLORS["accent_blue"],
                    font=FONTS["body_bold"],
                    relief="flat",
                    width=8,
                    pady=3,
                    cursor="hand2",
                )
                btn_dn.pack(side=tk.LEFT, padx=3)
                self.floor_btns[("down", fl)] = btn_dn

    def _build_door_card(self, parent: tk.Misc) -> None:
        """Tạo nhóm nút điều khiển đóng/mở cửa, giả lập kẹt IR X0 và E-Stop X1."""
        card = self._make_card(parent, "ĐIỀU KHIỂN CỬA & NGẮT AN TOÀN (EXTI)", COLORS["accent_green"])
        row = tk.Frame(card, bg=COLORS["bg_card"])
        row.pack(fill=tk.X)

        buttons = [
            ("Mở cửa", "elev door open", COLORS["btn_idle"], COLORS["text_primary"]),
            ("Đóng cửa", "elev door close", COLORS["btn_idle"], COLORS["text_primary"]),
            ("Kẹt cửa IR (X0)", "elev door obstruct", COLORS["accent_amber"], "#111827"),
            ("E-STOP (X1)", "elev fault estop", COLORS["accent_red"], "#FFFFFF"),
        ]
        for label, cmd, bg, fg in buttons:
            tk.Button(
                row,
                text=label,
                command=lambda c=cmd: self.dispatch_command(c),
                bg=bg,
                fg=fg,
                font=FONTS["body_bold"],
                relief="flat",
                pady=5,
                cursor="hand2",
            ).pack(side=tk.LEFT, expand=True, fill=tk.X, padx=2)

    def _build_hil_and_sensors_card(self, parent: tk.Misc) -> None:
        """Tạo thanh kéo cảm biến Offline (LM35 / Tải trọng) và nút tiêm lỗi HIL."""
        card = self._make_card(parent, "TIÊM LỖI HIL & MÔ PHỎNG CẢM BIẾN", COLORS["accent_amber"])

        # Thanh trượt giả lập cảm biến (dùng tức thời khi chạy Offline Simulator)
        sliders = tk.Frame(card, bg=COLORS["bg_card_alt"], padx=8, pady=6)
        sliders.pack(fill=tk.X, pady=(0, 8))

        self.temp_var = tk.DoubleVar(value=27.0)
        self.load_var = tk.IntVar(value=150)

        self.temp_val_lbl = tk.Label(
            sliders, text="LM35 (PC2): 27.0 °C", fg=COLORS["text_primary"], bg=COLORS["bg_card_alt"], font=FONTS["mono"]
        )
        self.temp_val_lbl.pack(anchor="w")
        tk.Scale(
            sliders,
            from_=20,
            to=80,
            resolution=0.5,
            orient=tk.HORIZONTAL,
            variable=self.temp_var,
            command=self._on_temp_slider_change,
            bg=COLORS["bg_card_alt"],
            fg=COLORS["text_muted"],
            troughcolor="#0F172A",
            highlightthickness=0,
            showvalue=False,
        ).pack(fill=tk.X, pady=(0, 6))

        self.load_val_lbl = tk.Label(
            sliders, text="Tải trọng (PC1): 150 kg", fg=COLORS["text_primary"], bg=COLORS["bg_card_alt"], font=FONTS["mono"]
        )
        self.load_val_lbl.pack(anchor="w")
        tk.Scale(
            sliders,
            from_=0,
            to=1000,
            resolution=10,
            orient=tk.HORIZONTAL,
            variable=self.load_var,
            command=self._on_load_slider_change,
            bg=COLORS["bg_card_alt"],
            fg=COLORS["text_muted"],
            troughcolor="#0F172A",
            highlightthickness=0,
            showvalue=False,
        ).pack(fill=tk.X)

        # 4 nút tiêm lỗi HIL nhanh 1-click
        fault_row = tk.Frame(card, bg=COLORS["bg_card"])
        fault_row.pack(fill=tk.X)
        faults = [
            ("Báo cháy (>55°C)", "elev fault fire", COLORS["accent_red"], "#FFFFFF"),
            ("Quá tải (>800kg)", "elev fault overload", COLORS["accent_orange"], "#FFFFFF"),
            ("Đứt CAN Link", "elev fault can_drop", "#B91C1C", "#FFFFFF"),
            ("Xóa lỗi / Phục hồi", "elev fault clear", COLORS["accent_green"], "#052E16"),
        ]
        for text, cmd, bg, fg in faults:
            tk.Button(
                fault_row,
                text=text,
                command=lambda c=cmd: self.dispatch_command(c),
                bg=bg,
                fg=fg,
                font=FONTS["body_bold"],
                relief="flat",
                pady=5,
                cursor="hand2",
            ).pack(side=tk.LEFT, expand=True, fill=tk.X, padx=2)

    def _build_metrics_card(self, parent: tk.Misc) -> None:
        """Dựng 8 thẻ thông số thời gian thực + thanh tiến độ mở cửa ở đầu cột phải."""
        card = self._make_card(parent, "GIÁM SÁT TELEMETRY THỜI GIAN THỰC", COLORS["accent_blue"])

        grid = tk.Frame(card, bg=COLORS["bg_card"])
        grid.pack(fill=tk.X)
        grid.grid_columnconfigure(0, weight=1)
        grid.grid_columnconfigure(1, weight=1)

        specs = [
            ("state", "TRẠNG THÁI HSM", "IDLE_CLOSED"),
            ("floor", "TẦNG / MỤC TIÊU", "F1 -> F0 (Dừng)"),
            ("sensors", "LM35 & TẢI TRỌNG", "27.0 °C | 150 kg"),
            ("can_exti", "CAN1 & ĐỘ TRỄ EXTI", "TEC=0 REC=0 | 0 us"),
            ("safety", "CỜ AN TOÀN", "Bình thường"),
            ("stats", "THỐNG KÊ HIL", "Chuyến: 0 | Mẫu: 0"),
        ]
        for idx, (key, label, init_val) in enumerate(specs):
            box = tk.Frame(
                grid,
                bg=COLORS["bg_card_alt"],
                highlightthickness=1,
                highlightbackground=COLORS["border"],
                padx=8,
                pady=6,
            )
            box.grid(row=idx // 2, column=idx % 2, sticky="ew", padx=3, pady=3)
            tk.Label(box, text=label, fg=COLORS["text_muted"], bg=COLORS["bg_card_alt"], font=("Segoe UI", 8)).pack(
                anchor="w"
            )
            val_lbl = tk.Label(box, text=init_val, fg=COLORS["text_primary"], bg=COLORS["bg_card_alt"], font=FONTS["mono_bold"])
            val_lbl.pack(anchor="w")
            self.metric_labels[key] = val_lbl

        # Thanh progress bar trực quan thể hiện độ mở cửa (0..100%)
        door_row = tk.Frame(card, bg=COLORS["bg_card"], pady=4)
        door_row.pack(fill=tk.X, pady=(4, 0))
        self.door_pct_lbl = tk.Label(
            door_row, text="Độ mở cửa: 0%", fg=COLORS["text_muted"], bg=COLORS["bg_card"], font=FONTS["mono"]
        )
        self.door_pct_lbl.pack(side=tk.LEFT)
        self.door_bar = ttk.Progressbar(door_row, orient=tk.HORIZONTAL, mode="determinate", maximum=100)
        self.door_bar.pack(side=tk.RIGHT, fill=tk.X, expand=True, padx=(10, 0))

    def _build_console_card(self, parent: tk.Misc) -> None:
        """Dựng khung nhật ký màu theo mức độ + ô nhập lệnh Shell CLI trực tiếp."""
        card = tk.Frame(
            parent,
            bg=COLORS["bg_card"],
            highlightthickness=1,
            highlightbackground=COLORS["border"],
            padx=10,
            pady=8,
        )
        card.pack(fill=tk.BOTH, expand=True)

        top = tk.Frame(card, bg=COLORS["bg_card"])
        top.pack(fill=tk.X, pady=(0, 6))
        tk.Label(top, text="NHẬT KÝ HỆ THỐNG & ZEPHYR SHELL", fg=COLORS["accent_blue"], bg=COLORS["bg_card"], font=FONTS["section"]).pack(
            side=tk.LEFT
        )
        tk.Button(
            top,
            text="Xóa Log",
            command=lambda: self.console.delete("1.0", tk.END),
            bg=COLORS["btn_idle"],
            fg=COLORS["text_muted"],
            font=("Segoe UI", 8),
            relief="flat",
            padx=6,
            cursor="hand2",
        ).pack(side=tk.RIGHT)

        self.console = tk.Text(
            card,
            bg=COLORS["console_bg"],
            fg="#CBD5E1",
            font=FONTS["mono"],
            relief="flat",
            padx=8,
            pady=6,
            wrap=tk.WORD,
        )
        self.console.pack(fill=tk.BOTH, expand=True)

        # Khai báo màu chữ riêng cho từng loại log để dễ nhìn khi debug
        self.console.tag_config("TX", foreground="#38BDF8")
        self.console.tag_config("SIM", foreground="#A78BFA")
        self.console.tag_config("SHELL", foreground="#94A3B8")
        self.console.tag_config("OK", foreground="#34D399")
        self.console.tag_config("WARN", foreground="#FBBF24")
        self.console.tag_config("ERR", foreground="#F87171")
        self.console.tag_config("INFO", foreground="#CBD5E1")

        # Thanh nhập lệnh Shell trực tiếp xuống bo mạch (hỗ trợ nhấn phím Enter)
        cmd_row = tk.Frame(card, bg=COLORS["bg_card"], pady=6)
        cmd_row.pack(fill=tk.X, pady=(6, 0))
        self.cmd_entry = tk.Entry(
            cmd_row,
            bg=COLORS["console_bg"],
            fg=COLORS["text_primary"],
            insertbackground=COLORS["text_primary"],
            font=FONTS["mono"],
            relief="flat",
        )
        self.cmd_entry.insert(0, "elev status")
        self.cmd_entry.pack(side=tk.LEFT, fill=tk.X, expand=True, ipady=5, padx=(0, 6))
        self.cmd_entry.bind("<Return>", lambda _e: self._on_send_manual_cmd())

        tk.Button(
            cmd_row,
            text="Gửi CLI",
            command=self._on_send_manual_cmd,
            bg=COLORS["accent_indigo"],
            fg="#FFFFFF",
            font=FONTS["body_bold"],
            relief="flat",
            padx=12,
            pady=4,
            cursor="hand2",
        ).pack(side=tk.RIGHT)

    # ==========================================================================
    # 2. XỬ LÝ SỰ KIỆN & ĐIỀU PHỐI LỆNH (LIVE STM32 vs OFFLINE SIMULATOR)
    # ==========================================================================
    def append_log(self, tag: str, msg: str) -> None:
        """Ghi log kèm dấu thời gian vào Console an toàn từ mọi luồng nhờ `root.after(0, ...)`."""
        ts = time.strftime("%H:%M:%S")

        def _insert() -> None:
            self.console.insert(tk.END, f"[{ts}] [{tag:<5}] {msg}\n", tag)
            self.console.see(tk.END)

        self.root.after(0, _insert)

    def dispatch_command(self, cmd: str) -> None:
        """Chuyển tiếp lệnh xuống STM32 qua Serial nếu đang Live, hoặc chạy ngay trên Simulator Offline."""
        self.append_log("TX", cmd)
        if self.live_mode and self.serial.is_connected:
            if cmd == "elev fault clear":
                if self._temp_tx_after_id is not None:
                    self.root.after_cancel(self._temp_tx_after_id)
                    self._temp_tx_after_id = None
                if self._load_tx_after_id is not None:
                    self.root.after_cancel(self._load_tx_after_id)
                    self._load_tx_after_id = None
                self._sync_sensor_widgets(27.0, 150)
            elif cmd == "elev fault fire":
                self._sync_sensor_widgets(62.0, int(self.load_var.get()))
            elif cmd == "elev fault overload":
                self._sync_sensor_widgets(float(self.temp_var.get()), 920)
            self.serial.send_cmd(cmd)
        else:
            self.sim.execute_cli(cmd)
            if cmd in ("elev fault fire", "elev fault overload", "elev fault clear"):
                self._sync_sensor_widgets(self.sim.temp_c, self.sim.load_kg)

    def _sync_sensor_widgets(self, temp_c: float, load_kg: int) -> None:
        """Đồng bộ hai thanh trượt LM35 và tải trọng mà không kích hoạt gửi lặp lệnh Serial."""
        self._suppress_slider_cb = True
        try:
            self.temp_var.set(temp_c)
            self.load_var.set(load_kg)
            self.temp_val_lbl.config(text=f"LM35 (PC2): {temp_c:.1f} °C")
            self.load_val_lbl.config(text=f"Tải trọng (PC1): {load_kg} kg")
        finally:
            self._suppress_slider_cb = False

    def _on_send_manual_cmd(self) -> None:
        """Gửi lệnh người dùng tự gõ từ ô nhập liệu Shell."""
        cmd = self.cmd_entry.get().strip()
        if not cmd:
            return
        self.dispatch_command(cmd)
        if not self.live_mode and not cmd.startswith("elev "):
            self.append_log("WARN", "Lệnh `bkit ...` chỉ chạy khi đã kết nối board STM32 qua RS232.")

    def _flush_live_temp_override(self) -> None:
        self._temp_tx_after_id = None
        if self.live_mode and self.serial.is_connected:
            deci_c = int(round(float(self.temp_var.get()) * 10.0))
            self.dispatch_command(f"elev sensor temp {deci_c}")

    def _flush_live_load_override(self) -> None:
        self._load_tx_after_id = None
        if self.live_mode and self.serial.is_connected:
            load_kg = int(self.load_var.get())
            self.dispatch_command(f"elev sensor load {load_kg}")

    def _on_temp_slider_change(self, _val: str) -> None:
        """Cập nhật thanh kéo nhiệt độ LM35 cho cả Offline Simulator lẫn Live STM32 (HIL Override)."""
        temp_c = float(self.temp_var.get())
        self.temp_val_lbl.config(text=f"LM35 (PC2): {temp_c:.1f} °C")
        if self._suppress_slider_cb:
            return
        self._last_user_slider_ts = time.monotonic()
        if self.live_mode and self.serial.is_connected:
            if self._temp_tx_after_id is not None:
                self.root.after_cancel(self._temp_tx_after_id)
            self._temp_tx_after_id = self.root.after(120, self._flush_live_temp_override)
        else:
            self.sim.update_sensors(temp_c, int(self.load_var.get()))

    def _on_load_slider_change(self, _val: str) -> None:
        """Cập nhật thanh kéo tải trọng cho cả Offline Simulator lẫn Live STM32 (HIL Override)."""
        load_kg = int(self.load_var.get())
        self.load_val_lbl.config(text=f"Tải trọng (PC1): {load_kg} kg")
        if self._suppress_slider_cb:
            return
        self._last_user_slider_ts = time.monotonic()
        if self.live_mode and self.serial.is_connected:
            if self._load_tx_after_id is not None:
                self.root.after_cancel(self._load_tx_after_id)
            self._load_tx_after_id = self.root.after(120, self._flush_live_load_override)
        else:
            self.sim.update_sensors(float(self.temp_var.get()), load_kg)

    def _refresh_ports(self) -> None:
        """Quét lại danh sách cổng COM mới cắm vào máy tính."""
        ports = self.serial.list_ports()
        self.port_combo["values"] = ports
        self.port_var.set(ports[0])
        self.append_log("INFO", f"Danh sách cổng Serial tìm thấy: {', '.join(ports)}")

    def _toggle_serial_connection(self) -> None:
        """Đóng/mở kết nối Serial RS232 và tự động hoán đổi giữa Live Mode <-> Offline Mode."""
        if self.serial.is_connected:
            self.serial.disconnect()
            self.live_mode = False
            self.btn_conn.config(text="Kết nối STM32", bg=COLORS["accent_indigo"])
            self.mode_badge.config(text="● OFFLINE SIMULATOR", bg=COLORS["accent_green"], fg="#052E16")
            self.append_log("INFO", "Đã ngắt Serial -> Quay lại chế độ OFFLINE SIMULATOR.")
            return

        port = self.port_var.get().strip()
        try:
            self.serial.connect(port, 115200)
            self.live_mode = True
            self.btn_conn.config(text="Ngắt RS232", bg=COLORS["accent_red"])
            self.mode_badge.config(text=f"● LIVE STM32 ({port})", bg=COLORS["accent_blue"], fg="#082F49")
            self.append_log("OK", f"Đã kết nối {port} @ 115200 bps và bật luồng $TWIN.")
        except Exception as exc:
            self.append_log("ERR", f"Không mở được cổng {port}: {exc}")

    def _start_smoke_suite(self) -> None:
        """Chạy chuỗi kiểm thử tự động 7 tầng (chạy được cả ở Live lẫn Offline)."""
        self.serial.run_smoke_suite(
            dispatch_cb=lambda cmd: self.root.after(0, lambda c=cmd: self.dispatch_command(c)),
            log_cb=self.append_log,
        )

    def _export_csv(self) -> None:
        """Xuất toàn bộ mẫu telemetry thu thập được ra file CSV."""
        fname = self.serial.export_csv()
        if fname is None:
            self.append_log("WARN", "Chưa có dữ liệu telemetry nào để xuất CSV.")
        else:
            self.append_log("OK", f"Đã xuất {len(self.serial.hil_log_rows)} dòng dữ liệu -> {fname}")

    # ==========================================================================
    # 3. VÒNG LẶP CẬP NHẬT GIAO DIỆN ĐỊNH KỲ (50 MS / FRAME)
    # ==========================================================================
    def _update_button_highlights(self, tel: Dict[str, Any]) -> None:
        """Đổi màu sáng đèn các nút tầng F1..F7 đang nằm trong hàng đợi xử lý."""
        cm = int(tel.get("cm", 0))
        um = int(tel.get("um", 0))
        dm = int(tel.get("dm", 0))

        for (kind, fl), btn in self.floor_btns.items():
            bit = 1 << (fl - 1)
            if kind == "cabin":
                on = (cm & bit) != 0
                btn.config(
                    bg=COLORS["btn_cabin_on"] if on else COLORS["btn_idle"],
                    fg="#FFFFFF" if on else COLORS["text_primary"],
                )
            elif kind == "up":
                on = (um & bit) != 0
                btn.config(
                    bg=COLORS["btn_up_on"] if on else COLORS["btn_idle"],
                    fg="#FFFFFF" if on else COLORS["accent_green"],
                )
            elif kind == "down":
                on = (dm & bit) != 0
                btn.config(
                    bg=COLORS["btn_dn_on"] if on else COLORS["btn_idle"],
                    fg="#FFFFFF" if on else COLORS["accent_blue"],
                )

    def _update_metrics(self, tel: Dict[str, Any]) -> None:
        """Làm mới 6 thẻ thông số telemetry và thanh progress độ mở cửa."""
        st = str(tel.get("st", "IDLE_CLOSED"))
        vi_name, st_color, _ = STATE_META.get(st, (st, COLORS["text_primary"], "--"))

        self.metric_labels["state"].config(text=f"{vi_name} ({st})", fg=st_color)

        d = int(tel.get("dir", 0))
        dir_txt = "LÊN ▲" if d > 0 else ("XUỐNG ▼" if d < 0 else "ĐỨNG YÊN")
        self.metric_labels["floor"].config(text=f"F{tel.get('fl', 1)} -> F{tel.get('tgt', 0)} ({dir_txt})")

        temp_c = float(tel.get("temp", 270)) / 10.0
        load_kg = int(tel.get("load", 0))
        self.metric_labels["sensors"].config(
            text=f"{temp_c:.1f} °C  |  {load_kg} kg",
            fg=COLORS["accent_red"] if temp_c >= 55.0 else COLORS["text_primary"],
        )

        self.metric_labels["can_exti"].config(
            text=f"TEC={tel.get('tec', 0)} REC={tel.get('rec', 0)} | {tel.get('lat_us', 0)} us"
        )

        flags = []
        if int(tel.get("fire", 0)):
            flags.append("CHÁY")
        if int(tel.get("estop", 0)):
            flags.append("E-STOP")
        if int(tel.get("ovl", 0)):
            flags.append("QUÁ TẢI")
        if int(tel.get("canf", 0)):
            flags.append("MẤT CAN")
        self.metric_labels["safety"].config(
            text=", ".join(flags) if flags else "An toàn (OK)",
            fg=COLORS["accent_red"] if flags else COLORS["accent_green"],
        )

        self.metric_labels["stats"].config(
            text=f"Chuyến: {tel.get('trips', 0)}  |  Mẫu CSV: {len(self.serial.hil_log_rows)}"
        )

        door_pct = int(tel.get("door", 0))
        self.door_pct_lbl.config(text=f"Độ mở cửa: {door_pct}%")
        self.door_bar["value"] = door_pct

    def _tick_loop(self) -> None:
        """Nhịp lặp không chặn 50 ms: rút dữ liệu Live hoặc bước Offline Simulator rồi vẽ lại UI."""
        # 1. Rút toàn bộ phản hồi Shell từ STM32 (nếu có) để hiển thị ra Console
        while not self.serial.shell_queue.empty():
            self.append_log("SHELL", self.serial.shell_queue.get_nowait())

        # 2. Chọn nguồn telemetry tùy chế độ Live hay Offline
        if self.live_mode and self.serial.is_connected:
            now_mono = time.monotonic()
            got_new_packet = False
            while not self.serial.telemetry_queue.empty():
                msg = self.serial.telemetry_queue.get_nowait()
                msg.setdefault("obstruct_flash", 0)
                self.latest_tel = msg
                got_new_packet = True
                self._last_live_rx_ts = now_mono

            # Tự động bật lại luồng $TWIN nếu quá 1.8 giây chưa thấy gói tin (vd: vừa nhấn nút RST trên mạch hoặc cắm cáp trễ)
            if (now_mono - self._last_live_rx_ts) > 1.8 and (now_mono - self._last_keepalive_tx_ts) > 2.0:
                self._last_keepalive_tx_ts = now_mono
                self.serial.send_cmd("elev twin on")

            # Nội suy mượt vị trí cabin (20 FPS) khi đang chạy Live trên STM32
            fl_int = float(self.latest_tel.get("fl", 1))
            live_dir = int(self.latest_tel.get("dir", 0))
            live_st = str(self.latest_tel.get("st", "IDLE_CLOSED"))
            if live_st in ("MOVING_UP", "MOVING_DOWN") or (live_st == "FIRE_EVAC" and live_dir < 0):
                step_f = SIM_TICK_MS / 1500.0
                if live_dir > 0:
                    if self._live_pos_f < fl_int or self._live_pos_f > fl_int + 1.0:
                        self._live_pos_f = fl_int
                    self._live_pos_f = min(float(MAX_FLOOR), fl_int + 0.94, self._live_pos_f + step_f)
                elif live_dir < 0:
                    if self._live_pos_f > fl_int or self._live_pos_f < fl_int - 1.0:
                        self._live_pos_f = fl_int
                    self._live_pos_f = max(float(MIN_FLOOR), fl_int - 0.94, self._live_pos_f - step_f)
            else:
                self._live_pos_f = fl_int
            self.latest_tel["pos_f"] = self._live_pos_f

            # Nội suy mượt 2 cánh cửa kính (20 FPS) khi đang chạy Live trên STM32
            raw_door = float(self.latest_tel.get("door", 0))
            if got_new_packet:
                self._live_door_pct = raw_door
            elif live_st == "DOOR_OPENING" or (
                live_st == "FAIL_SAFE"
                and int(self.latest_tel.get("canf", 0))
                and not int(self.latest_tel.get("estop", 0))
                and self._live_door_pct < 100.0
            ):
                self._live_door_pct = min(100.0, self._live_door_pct + (SIM_TICK_MS * 100.0 / 700.0))
            elif live_st == "DOOR_CLOSING" or (live_st == "FIRE_EVAC" and fl_int > MIN_FLOOR and self._live_door_pct > 0):
                self._live_door_pct = max(0.0, self._live_door_pct - (SIM_TICK_MS * 100.0 / 700.0))
            self.latest_tel["door"] = int(round(self._live_door_pct))

            # Đồng bộ ngược 2 chiều: khi xoay VR1 / hơ LM35 / bấm phím trên mạch, thanh kéo trên PC tự chạy theo!
            if (
                got_new_packet
                and self._temp_tx_after_id is None
                and self._load_tx_after_id is None
                and (time.monotonic() - self._last_user_slider_ts) > 0.45
            ):
                hw_temp = round((float(self.latest_tel.get("temp", 270)) / 10.0) * 2.0) / 2.0
                hw_load = int(round(int(self.latest_tel.get("load", 150)) / 10.0) * 10)
                hw_temp = max(20.0, min(80.0, hw_temp))
                hw_load = max(0, min(1000, hw_load))
                self._sync_sensor_widgets(hw_temp, hw_load)
        else:
            self.latest_tel = self.sim.step(SIM_TICK_MS)
            # Lưu mẫu mô phỏng mỗi 200 ms (khớp chu kỳ $TWIN thật) để sẵn sàng Export CSV
            now_ms = int(self.latest_tel.get("ms", 0))
            if now_ms - self._last_sim_record_ms >= 200:
                self._last_sim_record_ms = now_ms
                self.serial.record_sample(self.latest_tel)

        # 3. Vẽ lại giếng thang, bảng đèn nút và các thẻ thông số
        self.shaft.redraw(self.latest_tel)
        self._update_button_highlights(self.latest_tel)
        self._update_metrics(self.latest_tel)

        self.root.after(UI_REFRESH_MS, self._tick_loop)
